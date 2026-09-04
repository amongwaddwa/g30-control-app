#include <FlickerFreePrint.h>
#include <ComEVesc.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>
#include "EEPROMAnything.h"

// -----------------------------------------------------------------------------
// Types used in function signatures
// -----------------------------------------------------------------------------
// Keep this BEFORE Arduino's generated function prototypes. ESP32 Arduino Core
// 3.3.x can otherwise generate parseRemoteRideConfig()/activateRemoteRideConfig()
// prototypes before it knows what RemoteRideConfig is.
struct RemoteRideConfig {
  float driveA;
  float fwA;
  float releaseRegenA;
  float batteryMaxA;
  float batteryRegenA;
  bool police;
  float policeSpeed;
  bool regenAbs;
  bool releaseEBrake;
  float rampAps;
  float cutoffStartV;
  float cutoffEndV;
};

// Explicit prototypes stop the Arduino preprocessor from inventing invalid ones.
bool parseRemoteRideConfig(const String &packet, RemoteRideConfig &cfg);
void activateRemoteRideConfig(const RemoteRideConfig &cfg);


// -----------------------------------------------------------------------------
// Ninebot G30 Max + ESP32-2432S028R (Cheap Yellow Display) + VESC/UBOX
//
// UART to VESC:
//   ESP32 GPIO22 (RX) <- VESC TX
//   ESP32 GPIO27 (TX) -> VESC RX
//
// Ninebot throttle:
//   Signal -> ESP32 GPIO35 (P3 connector)
//   GND    -> ESP32 GND
//   Supply -> use the correct throttle supply for your hardware
//
// IMPORTANT: GPIO35 must NEVER see more than 3.3 V.
// For extra fail-safe protection, a 10k-47k resistor from GPIO35 to GND is
// recommended so a disconnected throttle signal cannot float.
// -----------------------------------------------------------------------------

// --------------------------- Display / vehicle data ---------------------------
float trip;
float startup_total_km;
float last_total_km_stored;
float total_km;
float tacho;
float rpm;
float speed;
float watts;
float wheel_diameter;
int maxspeed;
int brightness = 255;

// Font settings
#define SPEEDFONT &JerseyM54_82pt7b
#define DATAFONTSMALL2 &JerseyM54_14pt7b
#define DATAFONTSMALL &JerseyM54_18pt7b
#define DATAFONTSMALLTEXT &Blockletter8pt7b

// Ninebot G30 motor/wheel parameters
#define MOTOR_POLES 30
#define WHEEL_DIAMETER_MM 254
#define GEAR_RAITO 1.0
#define PI 3.141592

// UART pins
#define RXD2 22 // ESP32 RX <- VESC TX
#define TXD2 27 // ESP32 TX -> VESC RX

// CYD built-in pins
#define LDR_PIN 34
#define LCD_BACK_LIGHT_PIN 21

// ------------------------------- TOUCH --------------------------------------
// ESP32-2432S028R / Cheap Yellow Display uses a separate SPI bus for XPT2046.
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CLK  25

SPIClass touchSPI(HSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

// Raw touch calibration starting values for a typical CYD.
// If touch positions are mirrored on your exact board, the three booleans below
// are the only values that should need changing.
#define TOUCH_X_MIN 280
#define TOUCH_X_MAX 3860
#define TOUCH_Y_MIN 340
#define TOUCH_Y_MAX 3860
#define TOUCH_SWAP_XY 1
#define TOUCH_INVERT_X 0
#define TOUCH_INVERT_Y 1

// Main-screen hidden hotspot over the centre/lower Ninebot/Segway-logo area.
// It is deliberately generous so the logo is easy to tap while riding stopped.
#define LOGO_TOUCH_X1 105
#define LOGO_TOUCH_X2 215
#define LOGO_TOUCH_Y1 155
#define LOGO_TOUCH_Y2 230

// -------------------------- BLUETOOTH NAVIGATION -----------------------------
// Nordic-UART-style BLE service. This matches the official ESP32 BLE UART
// example UUIDs, so a phone companion can write navigation packets to RX.
#define NAV_BLE_DEVICE_NAME "G30-NAV"
#define NAV_BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NAV_BLE_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NAV_BLE_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define NAV_STALE_MS 12000UL
#define NAV_MAX_ROUTE_POINTS 18
#define NAV_MAX_ROADS 16
#define NAV_MAX_ROAD_POINTS 10

// BLE writes can arrive faster than loop() can parse them. Queue them instead
// of overwriting one shared String, which used to drop map/road packets.
#define BLE_RX_QUEUE_DEPTH 16
#define BLE_RX_PACKET_MAX 190
#define BLE_PACKETS_PER_LOOP 4
#define BLE_PARSE_BUDGET_US 2500UL

// Batch road geometry redraws so 10-16 ROAD packets do not repaint the map
// 10-16 times in a row. This is the biggest navigation smoothness fix.
#define NAV_MAP_SETTLE_MS 100UL
#define NAV_MAP_MAX_REDRAW_MS 320UL

// Main-screen layout. Speed is shifted left and navigation gets the right side.
#define NAV_PANEL_X 166
#define NAV_PANEL_Y 42
#define NAV_PANEL_W 150
#define NAV_PANEL_H 143

// Phone -> display text protocol:
// NAV|TURN|DIST_M|ROAD|BEARING_DEG|ETA_MIN
// MAP|x,y;x,y;...            (0..100 coordinates, highlighted route)
// ROADCLR                     (clear cached nearby streets)
// ROAD|W|x,y;x,y;...         (nearby OSM road; W=1 local, W=2 major)
// CLR                         (clear active route + roads)
//
// TURN examples: L R SL SR U S ARRIVE
//
// The phone companion gets routing/map data from OpenStreetMap-based services.
// The ESP32 itself does NOT download internet map tiles over BLE.

// Throttle pin - GPIO35 is input-only and is exposed on P3
#define THROTTLE_PIN 35

// ------------------------------ THROTTLE SETUP -------------------------------
// V3: tuned for the user's measured Ninebot throttle range (~20 RAW released
// to ~230 RAW at full throttle), with conservative current for bench testing.
//
// IMPORTANT:
// - Start the ESP32 with the throttle COMPLETELY RELEASED.
// - The idle calibration is deliberately only allowed between RAW 5 and 80.
//   This prevents a throttle held open at startup from being learned as idle.
// - GPIO35 must never receive more than 3.3 V.
// - A physical 10k-47k pulldown from GPIO35 to GND is strongly recommended.
//
// The throttle signal must be on GPIO35.
// P3 on the CYD has NO power pin, so the throttle must be powered separately.

// Learn only the known released-throttle region, not arbitrary ADC values.
#define THROTTLE_IDLE_CAL_TIME_MS 300
#define THROTTLE_IDLE_STABILITY_RAW 100
#define THROTTLE_IDLE_MIN_RAW 500
#define THROTTLE_IDLE_MAX_RAW 3000

// Your measured Ninebot throttle voltage is about:
// released = 1132 mV
// full     = 3145 mV
#define THROTTLE_RELEASE_MARGIN_MV 120
#define THROTTLE_ARM_TIME_MS 250
#define THROTTLE_DIRECTION_DETECT_MV 150
#define THROTTLE_DEADZONE_MV 100
#define THROTTLE_FULL_MV 3145.0f

// Bench-test current. Keep this low until everything is proven stable.
#define HARD_MAX_DRIVE_CURRENT_A 35.0f
#define DEFAULT_MAX_DRIVE_CURRENT_A 25.0f
#define MIN_CURRENT_RISE_A_PER_SEC 20.0f
#define MAX_CURRENT_RISE_A_PER_SEC 120.0f
#define DEFAULT_CURRENT_RISE_A_PER_SEC 60.0f

// User settings controlled by the touchscreen and/or Android app.
// VESC Tool limits remain the final hard protection.
float userMaxDriveCurrentA = DEFAULT_MAX_DRIVE_CURRENT_A;
float userCurrentRiseAPerSec = DEFAULT_CURRENT_RISE_A_PER_SEC;

// "Police mode": soft speed limiter. OFF = no speed limit from this ESP32.
// ON = current is smoothly tapered from 18.0 km/h and reaches zero at 20.0 km/h.
bool policeMode = false;
#define MIN_POLICE_LIMIT_KMH 10.0f
#define MAX_POLICE_LIMIT_KMH 25.0f
#define DEFAULT_POLICE_LIMIT_KMH 20.0f
float userPoliceLimitKmh = DEFAULT_POLICE_LIMIT_KMH;

// Field Weakening UI range.
// The ESP32 sends this value to the VESC Lisp helper as a TEMPORARY config
// change. It is not written to the VESC flash on every slider movement.
#define HARD_MAX_FIELD_WEAKENING_A 35.0f
#define DEFAULT_FIELD_WEAKENING_A 0.0f
float userFieldWeakeningA = DEFAULT_FIELD_WEAKENING_A;
bool fieldWeakeningApplyPending = true;
bool maxCurrentApplyPending = true;
bool vescStoreApplyPending = false;
unsigned long lastVescLispCommandMs = 0;
#define VESC_LISP_MIN_INTERVAL_MS 600UL

// Adjustable regenerative e-brake when the throttle is fully released.
#define HARD_MAX_REGEN_BRAKE_A 12.0f
#define DEFAULT_REGEN_BRAKE_A 2.0f
float userRegenBrakeA = DEFAULT_REGEN_BRAKE_A;
bool regenAbsEnabled = true;
bool releaseEBrakeEnabled = true;

// Stock-G30-oriented battery limits exposed in the Android app.
// Values are deliberately clamped to conservative ranges.
#define MIN_BATTERY_CURRENT_MAX_A 5.0f
#define HARD_MAX_BATTERY_CURRENT_MAX_A 25.0f
#define DEFAULT_BATTERY_CURRENT_MAX_A 15.0f
float userBatteryCurrentMaxA = DEFAULT_BATTERY_CURRENT_MAX_A;

#define HARD_MAX_BATTERY_REGEN_A 8.0f
#define DEFAULT_BATTERY_REGEN_A 3.0f
float userBatteryRegenMaxA = DEFAULT_BATTERY_REGEN_A;

#define MIN_BATTERY_CUTOFF_START_V 30.0f
#define MAX_BATTERY_CUTOFF_START_V 36.0f
#define DEFAULT_BATTERY_CUTOFF_START_V 34.0f
float userBatteryCutoffStartV = DEFAULT_BATTERY_CUTOFF_START_V;

#define MIN_BATTERY_CUTOFF_END_V 28.0f
#define MAX_BATTERY_CUTOFF_END_V 33.0f
#define DEFAULT_BATTERY_CUTOFF_END_V 30.0f
float userBatteryCutoffEndV = DEFAULT_BATTERY_CUTOFF_END_V;

#define E_BRAKE_MIN_SPEED_KMH 3.0f
#define E_BRAKE_FULL_STRENGTH_KMH 8.0f
#define E_BRAKE_VOLTAGE_TAPER_START 40.5f
#define E_BRAKE_VOLTAGE_CUTOFF 41.2f

// Display-side rear-wheel anti-lock modulation for regenerative braking.
// This is separate from VESC's ABS-over-current protection setting.
#define REGEN_ABS_DECEL_TRIGGER_KMHPS 55.0f
#define REGEN_ABS_SPEED_DROP_KMH 3.2f
#define REGEN_ABS_RELEASE_MS 110
#define REGEN_ABS_REAPPLY_MS 180
bool eBrakeActive = false;
float filteredWheelDecelKmhps = 0.0f;
float previousAbsSpeedKmh = 0.0f;
unsigned long previousAbsSpeedMs = 0;
unsigned long regenAbsReleaseUntilMs = 0;

// Manual Save & Restart flow.
bool settingsDirty = false;
bool saveRestartInProgress = false;
bool saveBlockedMessage = false;
unsigned long saveRestartStartedMs = 0;
unsigned long saveBlockedUntilMs = 0;
#define SAVE_RESTART_DELAY_MS 3200

// Voltage-based signal plausibility. This avoids false faults caused by
// different ESP32 raw ADC scaling with 11 dB attenuation.
#define THROTTLE_BAD_LOW_MV 250
#define THROTTLE_BAD_HIGH_MV 3290
#define THROTTLE_BAD_TIME_MS 300
#define THROTTLE_MAX_STEP_MV 1500

// Control and telemetry timing. Motor commands do NOT depend on telemetry RX.
#define CONTROL_INTERVAL_MS 20
#define KEEPALIVE_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 60
#define DISPLAY_INTERVAL_MS 25
#define DEBUG_INTERVAL_MS 500

// ------------------------------- WARNINGS ------------------------------------
int EEPROM_MAGIC_VALUE = 0;
#define EEPROM_UPDATE_EACH_KM 0.1

int COLOR_WARNING_SPEED = TFT_RED;
#define HIGH_SPEED_WARNING 60

int COLOR_WARNING_TEMP_VESC = TFT_WHITE;
#define VESC_TEMP_WARNING1 50
#define VESC_TEMP_WARNING2 80

int COLOR_WARNING_TEMP_MOTOR = TFT_WHITE;
#define MOTOR_TEMP_WARNING1 80
#define MOTOR_TEMP_WARNING2 120

// 10S / 36V Ninebot G30 battery display warnings
int BATTERY_WARNING_COLOR = TFT_WHITE;
#define BATTERY_WARNING_HIGH 42.2
#define BATTERY_WARNING_LOW 33.0
#define BATTERY_WARNING_0 30.0

int ERROR_WARNING_COLOR = TFT_WHITE;

#define DO_LOGO_DRAW
// #define DEBUG_MODE  // enable only while diagnosing over USB

#ifdef DO_LOGO_DRAW
#include <PNGdec.h>
#include "startup_image.h"
#include "background_image.h"
PNG png;
int16_t xpos = 0;
int16_t ypos = 0;
#define MAX_IMAGE_WDITH 320
#endif

// ComEVesc default timeout is 100 ms. A shorter timeout prevents a lost VESC
// response from blocking throttle updates for too long.
ComEVesc UART(20);
HardwareSerial VescSerial(1);


TFT_eSPI tft = TFT_eSPI();

// Original SimpleVescDisplay rendering method.
// FlickerFreePrint repaints only changed characters in BLACK before drawing
// the new value. This avoids white/old-glyph bars with the proportional
// JerseyM54 FreeFonts.
FlickerFreePrint<TFT_eSPI> ClassicSpeed(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicVescTemp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicMotorTemp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicBatteryV(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicPower(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicPhaseA(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicBattA(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> ClassicOdometer(&tft, TFT_WHITE, TFT_BLACK);


// ----------------------------- Throttle state --------------------------------
int throttleRaw = 0;
int throttleMv = 0;
int throttlePrevRaw = -1;
float throttleFiltered = 0.0f;
bool throttleFilterInitialized = false;

bool throttleIdleCalibrated = false;
int throttleIdleRaw = 0;
int throttleIdleMv = 0;
long throttleCalSum = 0;
int throttleCalSamples = 0;
int throttleCalMin = 4095;
int throttleCalMax = 0;
unsigned long throttleCalStartMs = 0;

bool throttleArmed = false;
bool throttleSignalFault = false;
String throttleFaultReason = "NONE";
int throttleDirection = 0; // 0 unknown, +1 raw rises with throttle, -1 raw falls
float throttlePercent = 0.0f;
unsigned long throttleReleasedSinceMs = 0;
unsigned long throttleBadSinceMs = 0;

bool vescSeen = false;
unsigned long lastVescRxMs = 0;
float commandedCurrentA = 0.0f;

unsigned long lastControlMs = 0;
unsigned long lastKeepaliveMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastDebugMs = 0;

// -------------------------- BLE NAVIGATION STATE ------------------------------
BLEServer *navBleServer = nullptr;
BLECharacteristic *navBleTx = nullptr;
volatile bool navBleConnected = false;
bool navBleWasConnected = false;

struct NavRoutePoint {
  int8_t x;
  int8_t y;
};

struct NavRoad {
  uint8_t count;
  uint8_t weight; // 1 local street, 2 larger road
  NavRoutePoint point[NAV_MAX_ROAD_POINTS];
};

NavRoad navRoads[NAV_MAX_ROADS];
uint8_t navRoadCount = 0;

struct NavigationState {
  bool active;
  char turn[9];
  char road[32];
  int distanceM;
  int bearingDeg;       // relative direction to destination, 0 = straight ahead
  int etaMin;
  int remainingM;
  char arrival[6];      // HH:MM supplied by phone
  unsigned long lastUpdateMs;
  uint8_t routeCount;
  NavRoutePoint route[NAV_MAX_ROUTE_POINTS];
};

NavigationState navState = {
  false, "S", "", 0, 0, 0, 0, "--:--", 0, 0, {}
};

char phoneTime[6] = "--:--";
unsigned long lastPhoneTimeMs = 0;

// Original SimpleVescDisplay-style dashboard redraw state.
bool classicDashboardNeedsRedraw = true;
#define ESTIMATED_FULL_RANGE_KM 40.0f

struct BleRxPacket {
  uint16_t len;
  char data[BLE_RX_PACKET_MAX];
};
QueueHandle_t navRxQueue = nullptr;
volatile uint32_t bleRxDroppedPackets = 0;

bool navMapDirty = true;
unsigned long navMapLastGeometryRxMs = 0;
unsigned long navMapLastDrawMs = 0;
unsigned long lastAppTelemetryMs = 0;
#define APP_TELEMETRY_INTERVAL_MS 200UL

// ------------------------------- UI STATE -----------------------------------
enum UiScreen {
  UI_MAIN = 0,
  UI_CONTROLLER,
  UI_TRIP,
  UI_BATTERY,
  UI_SETTINGS
};

UiScreen uiScreen = UI_MAIN;
bool uiNeedsFullRedraw = true;
unsigned long lastTouchMs = 0;
bool touchWasDown = false;
uint8_t activeSettingsSlider = 0; // 0 none, 1 current, 2 FW, 3 regen
bool settingsControlsDirty = false;
uint8_t settingsSection = 0; // 0 Performance, 1 Braking & Save
uint8_t settingsTransitionTarget = 0;
bool settingsSwipeCandidate = false;
int16_t settingsTouchStartX = 0;
int16_t settingsTouchStartY = 0;
int16_t settingsTouchLastX = 0;
int16_t settingsTouchLastY = 0;

// --------------------------- MENU ANIMATIONS ---------------------------------
// These transitions are fully non-blocking: throttle/UART control keeps running.
enum UiTransitionType {
  TRANS_NONE = 0,
  TRANS_OPEN_MENU,
  TRANS_PAGE_NEXT,
  TRANS_PAGE_PREV,
  TRANS_CLOSE_MENU,
  TRANS_SETTINGS_UP,
  TRANS_SETTINGS_DOWN
};

UiTransitionType uiTransition = TRANS_NONE;
UiScreen uiTransitionTarget = UI_MAIN;
unsigned long uiTransitionStartMs = 0;
unsigned long uiTransitionLastFrameMs = 0;
#define UI_TRANSITION_MS 135
#define UI_TRANSITION_FRAME_MS 16

// Smooth visual speedometer. Real "speed" remains untouched for safety/limiter.
float displaySpeed = 0.0f;
// Slow display-only voltage estimate for range/battery bars. Raw battery volts
// are still shown numerically and all safety logic still uses raw VESC data.
float displayBatteryVoltage = 0.0f;

// Session statistics
unsigned long sessionStartMs = 0;
unsigned long movingStartMs = 0;
unsigned long movingAccumMs = 0;
bool movingNow = false;
float sessionMaxSpeed = 0.0f;
float sessionTripKm = 0.0f;
float averageSpeedKmh = 0.0f;
unsigned long lastDistanceIntegrationMs = 0;
#define MAX_VALID_SPEED_KMH 120.0f

// Settings are stored away from odometer EEPROM bytes.
#define SETTINGS_EEPROM_ADDR 16
#define SETTINGS_MAGIC 0x47333139UL  // G30 V19
struct DisplaySettings {
  uint32_t magic;
  float maxCurrentA;
  float fieldWeakeningA;
  float regenBrakeA;
  float batteryCurrentMaxA;
  float batteryRegenMaxA;
  float policeLimitKmh;
  float currentRiseAPerSec;
  float batteryCutoffStartV;
  float batteryCutoffEndV;
  uint8_t police;
  uint8_t regenAbs;
  uint8_t releaseEBrake;
  uint8_t reserved;
};


// UI color helpers are implemented below in the original helper section.
uint16_t uiCardColor();
uint16_t uiCardBorder();
uint16_t uiAccent();
uint16_t uiOrange();

void applyControllerSettingsToVesc(bool storeToFlash);
bool controllerSafeToStore();
void saveDisplaySettings();
void requestSaveAndRestart(unsigned long now);
bool handleAdvancedAppPacket(const String &packet);
bool applyAdvancedVescSetting(const String &key, float value);
void sendVescLispExpression(const String &expr);
String vescNumber(float value, unsigned int decimals);

// -----------------------------------------------------------------------------
// BLE NAVIGATION
// -----------------------------------------------------------------------------

class NavServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    navBleConnected = true;
  }

  void onDisconnect(BLEServer *server) override {
    navBleConnected = false;
  }
};

class NavRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    if (navRxQueue == nullptr) return;

    String value = characteristic->getValue();
    if (value.length() == 0) return;

    BleRxPacket packet;
    size_t n = (size_t)value.length();
    if (n >= BLE_RX_PACKET_MAX) n = BLE_RX_PACKET_MAX - 1;
    memcpy(packet.data, value.c_str(), n);
    packet.data[n] = '\0';
    packet.len = (uint16_t)n;

    if (xQueueSend(navRxQueue, &packet, 0) != pdTRUE) {
      bleRxDroppedPackets++;
      BleRxPacket discarded;
      xQueueReceive(navRxQueue, &discarded, 0);
      xQueueSend(navRxQueue, &packet, 0);
    }
  }
};

void setupNavigationBle() {
  if (navRxQueue == nullptr) {
    navRxQueue = xQueueCreate(BLE_RX_QUEUE_DEPTH, sizeof(BleRxPacket));
  }

  BLEDevice::init(NAV_BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);

  navBleServer = BLEDevice::createServer();
  navBleServer->setCallbacks(new NavServerCallbacks());

  BLEService *service =
      navBleServer->createService(NAV_BLE_SERVICE_UUID);

  navBleTx = service->createCharacteristic(
      NAV_BLE_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  navBleTx->addDescriptor(new BLE2902());

  BLECharacteristic *rx = service->createCharacteristic(
      NAV_BLE_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  rx->setCallbacks(new NavRxCallbacks());

  service->start();

  BLEAdvertising *advertising = navBleServer->getAdvertising();
  advertising->addServiceUUID(NAV_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

#ifdef DEBUG_MODE
  Serial.println("BLE navigation advertising as G30-NAV");
#endif
}

void serviceNavigationBleConnection() {
  if (!navBleConnected && navBleWasConnected) {
    // Restart advertising without blocking the 50 Hz throttle loop.
    navBleServer->startAdvertising();
    navBleWasConnected = false;
  }

  if (navBleConnected && !navBleWasConnected) {
    navBleWasConnected = true;
  }
}

int navFieldEnd(const String &s, int start) {
  int p = s.indexOf('|', start);
  return (p < 0) ? s.length() : p;
}

String navField(const String &s, int fieldIndex) {
  int start = 0;
  for (int i = 0; i < fieldIndex; i++) {
    int p = s.indexOf('|', start);
    if (p < 0) return "";
    start = p + 1;
  }
  int end = navFieldEnd(s, start);
  return s.substring(start, end);
}

void copyNavText(char *dst, size_t dstLen, const String &src) {
  if (dstLen == 0) return;
  size_t n = (size_t)src.length();
  if (n > (dstLen - 1)) n = dstLen - 1;
  memcpy(dst, src.c_str(), n);
  dst[n] = '\0';
}

// -----------------------------------------------------------------------------
// ANDROID CONTROL APP PROTOCOL
// -----------------------------------------------------------------------------
// The native Android app uses the same BLE UART service as navigation.
//
// Phone -> ESP32:
//   CFG|GET
//   CFG|APPLY|drive|fw|releaseRegen|batteryMax|batteryRegen|police|
//             policeSpeed|regenAbs|releaseEBrake|ramp|cutStart|cutEnd
//   CFG|SAVE|...same values...
//   CFG|RESTART
//   CFG|PING
//
// ESP32 -> Phone notifications:
//   CFG|STATE|...
//   TEL|...
//   ACK|... / ERR|...

void bleNotifyLine(const String &line) {
  if (!navBleConnected || navBleTx == nullptr) return;
  navBleTx->setValue((uint8_t *)line.c_str(), line.length());
  navBleTx->notify();
}

void sendAppConfigState() {
  // One compact packet fits comfortably inside the negotiated 185-byte MTU.
  String s = "CFG|STATE|D=" + String(userMaxDriveCurrentA, 1);
  s += "|F=" + String(userFieldWeakeningA, 1);
  s += "|R=" + String(userRegenBrakeA, 1);
  s += "|BM=" + String(userBatteryCurrentMaxA, 1);
  s += "|BR=" + String(userBatteryRegenMaxA, 1);
  s += "|P=" + String(policeMode ? 1 : 0);
  s += "|PS=" + String(userPoliceLimitKmh, 1);
  s += "|A=" + String(regenAbsEnabled ? 1 : 0);
  s += "|E=" + String(releaseEBrakeEnabled ? 1 : 0);
  s += "|RA=" + String(userCurrentRiseAPerSec, 1);
  s += "|CS=" + String(userBatteryCutoffStartV, 1);
  s += "|CE=" + String(userBatteryCutoffEndV, 1);
  bleNotifyLine(s);
}

bool parseRemoteRideConfig(const String &packet, RemoteRideConfig &cfg) {
  cfg.driveA = navField(packet, 2).toFloat();
  cfg.fwA = navField(packet, 3).toFloat();
  cfg.releaseRegenA = navField(packet, 4).toFloat();
  cfg.batteryMaxA = navField(packet, 5).toFloat();
  cfg.batteryRegenA = navField(packet, 6).toFloat();
  cfg.police = navField(packet, 7).toInt() != 0;
  cfg.policeSpeed = navField(packet, 8).toFloat();
  cfg.regenAbs = navField(packet, 9).toInt() != 0;
  cfg.releaseEBrake = navField(packet, 10).toInt() != 0;
  cfg.rampAps = navField(packet, 11).toFloat();
  cfg.cutoffStartV = navField(packet, 12).toFloat();
  cfg.cutoffEndV = navField(packet, 13).toFloat();

  bool valid =
      cfg.driveA >= 5.0f && cfg.driveA <= HARD_MAX_DRIVE_CURRENT_A &&
      cfg.fwA >= 0.0f && cfg.fwA <= HARD_MAX_FIELD_WEAKENING_A &&
      cfg.releaseRegenA >= 0.0f && cfg.releaseRegenA <= HARD_MAX_REGEN_BRAKE_A &&
      cfg.batteryMaxA >= MIN_BATTERY_CURRENT_MAX_A &&
      cfg.batteryMaxA <= HARD_MAX_BATTERY_CURRENT_MAX_A &&
      cfg.batteryRegenA >= 0.0f &&
      cfg.batteryRegenA <= HARD_MAX_BATTERY_REGEN_A &&
      cfg.policeSpeed >= MIN_POLICE_LIMIT_KMH &&
      cfg.policeSpeed <= MAX_POLICE_LIMIT_KMH &&
      cfg.rampAps >= MIN_CURRENT_RISE_A_PER_SEC &&
      cfg.rampAps <= MAX_CURRENT_RISE_A_PER_SEC &&
      cfg.cutoffStartV >= MIN_BATTERY_CUTOFF_START_V &&
      cfg.cutoffStartV <= MAX_BATTERY_CUTOFF_START_V &&
      cfg.cutoffEndV >= MIN_BATTERY_CUTOFF_END_V &&
      cfg.cutoffEndV <= MAX_BATTERY_CUTOFF_END_V &&
      cfg.cutoffEndV <= (cfg.cutoffStartV - 1.0f);

  return valid;
}

void activateRemoteRideConfig(const RemoteRideConfig &cfg) {
  userMaxDriveCurrentA = cfg.driveA;
  userFieldWeakeningA = cfg.fwA;
  userRegenBrakeA = cfg.releaseRegenA;
  userBatteryCurrentMaxA = cfg.batteryMaxA;
  userBatteryRegenMaxA = cfg.batteryRegenA;
  policeMode = cfg.police;
  userPoliceLimitKmh = cfg.policeSpeed;
  regenAbsEnabled = cfg.regenAbs;
  releaseEBrakeEnabled = cfg.releaseEBrake;
  userCurrentRiseAPerSec = cfg.rampAps;
  userBatteryCutoffStartV = cfg.cutoffStartV;
  userBatteryCutoffEndV = cfg.cutoffEndV;

  settingsDirty = true;
  settingsControlsDirty = true;
  classicDashboardNeedsRedraw = true;
}


// -----------------------------------------------------------------------------
// ADVANCED ANDROID VESC SETTINGS
// -----------------------------------------------------------------------------
// Advanced parameters are deliberately sent one-at-a-time by the phone app.
// This avoids a huge BLE packet and respects the VESC Lisp REPL rate limit.
//
// Phone -> ESP32:
//   ADV|SET|KEY|VALUE
//   ADV|SAVEALL
//
// ESP32 -> Phone:
//   ACK|ADV_SET|KEY
//   ACK|ADV_SAVE|RESTARTING
//   ERR|ADV_BUSY / ERR|ADV_VALUE / ERR|ADV_KEY / ERR|STOP_FIRST
//
// These are a broad set of documented LispBM conf-set parameters that are
// useful for a scooter. Motor detection, FOC R/L/flux/KP/KI, sensor mode,
// controller app selection and CAN setup are intentionally NOT exposed here.

bool applyAdvancedVescSetting(const String &key, float value) {
  String param;
  float sendValue = value;
  unsigned int decimals = 3;

  if (key == "CURS") {
    if (value < 0.20f || value > 1.00f) return false;
    param = "l-current-max-scale";
  }
  else if (key == "BRS") {
    if (value < 0.20f || value > 1.00f) return false;
    param = "l-current-min-scale";
  }
  else if (key == "MINDUTY") {
    if (value < 0.0f || value > 0.05f) return false;
    param = "l-min-duty";
  }
  else if (key == "DUTY") {
    if (value < 0.60f || value > 0.95f) return false;
    param = "l-max-duty";
  }
  else if (key == "WMAX") {
    if (value < 250.0f || value > 1500.0f) return false;
    param = "l-watt-max";
    decimals = 0;
  }
  else if (key == "WBRAKE") {
    if (value < 0.0f || value > 500.0f) return false;
    param = "l-watt-min";
    sendValue = -value;
    decimals = 0;
  }
  else if (key == "ERPM") {
    if (value < 10000.0f || value > 30000.0f) return false;
    param = "l-max-erpm";
    decimals = 0;
  }
  else if (key == "ERPMS") {
    if (value < 0.50f || value > 0.95f) return false;
    param = "l-erpm-start";
  }
  else if (key == "VINMIN") {
    if (value < 28.0f || value > 34.0f) return false;
    param = "l-min-vin";
    decimals = 1;
  }
  else if (key == "VINMAX") {
    if (value < 40.0f || value > 45.0f) return false;
    param = "l-max-vin";
    decimals = 1;
  }
  else if (key == "TMSTART") {
    if (value < 60.0f || value > 90.0f) return false;
    param = "l-temp-motor-start";
    decimals = 0;
  }
  else if (key == "TMEND") {
    if (value < 75.0f || value > 105.0f) return false;
    param = "l-temp-motor-end";
    decimals = 0;
  }
  else if (key == "TACC") {
    if (value < 0.0f || value > 0.50f) return false;
    param = "l-temp-accel-dec";
  }
  else if (key == "FWDUTY") {
    if (value < 0.70f || value > 0.95f) return false;
    param = "foc-fw-duty-start";
  }
  else if (key == "VMAX") {
    // App uses km/h. VESC max-speed uses SI m/s.
    if (value < 10.0f || value > 60.0f) return false;
    param = "max-speed";
    sendValue = value / 3.6f;
  }
  else if (key == "WHEEL") {
    // App uses mm. VESC SI wheel diameter uses meters.
    if (value < 200.0f || value > 300.0f) return false;
    param = "si-wheel-diameter";
    sendValue = value / 1000.0f;
  }
  else if (key == "BATAH") {
    if (value < 5.0f || value > 30.0f) return false;
    param = "si-battery-ah";
    decimals = 1;
  }
  else if (key == "CELLS") {
    int cells = (int)roundf(value);
    if (cells < 8 || cells > 14) return false;
    param = "si-battery-cells";
    sendValue = (float)cells;
    decimals = 0;
  }
  else if (key == "POLES") {
    int poles = (int)roundf(value);
    if (poles < 2 || poles > 60 || (poles & 1)) return false;
    param = "si-motor-poles";
    sendValue = (float)poles;
    decimals = 0;
  }
  else if (key == "GEAR") {
    if (value < 0.50f || value > 10.0f) return false;
    param = "si-gear-ratio";
  }
  else {
    return false;
  }

  String cmd = "(trap (conf-set '" + param + " " +
               vescNumber(sendValue, decimals) + "))";
  sendVescLispExpression(cmd);
  lastVescLispCommandMs = millis();
  return true;
}

bool handleAdvancedAppPacket(const String &packet) {
  if (packet == "ADV|SAVEALL") {
    if (!controllerSafeToStore()) {
      bleNotifyLine("ERR|STOP_FIRST");
      return true;
    }

    unsigned long now = millis();
    if (lastVescLispCommandMs != 0 &&
        (now - lastVescLispCommandMs) < VESC_LISP_MIN_INTERVAL_MS) {
      bleNotifyLine("ERR|ADV_BUSY");
      return true;
    }

    UART.setCurrent(0.0f);
    commandedCurrentA = 0.0f;
    saveDisplaySettings();
    sendVescLispExpression("(conf-store)");
    lastVescLispCommandMs = now;
    settingsDirty = false;

    bleNotifyLine("ACK|ADV_SAVE|RESTARTING");
    saveRestartInProgress = true;
    saveRestartStartedMs = now;
    return true;
  }

  if (!packet.startsWith("ADV|SET|")) return false;

  if (!controllerSafeToStore()) {
    bleNotifyLine("ERR|STOP_FIRST");
    return true;
  }

  String key = navField(packet, 2);
  String raw = navField(packet, 3);
  if (key.length() == 0 || raw.length() == 0) {
    bleNotifyLine("ERR|ADV_VALUE");
    return true;
  }

  unsigned long now = millis();
  if (lastVescLispCommandMs != 0 &&
      (now - lastVescLispCommandMs) < VESC_LISP_MIN_INTERVAL_MS) {
    bleNotifyLine("ERR|ADV_BUSY");
    return true;
  }

  float value = raw.toFloat();
  if (!isfinite(value)) {
    bleNotifyLine("ERR|ADV_VALUE");
    return true;
  }

  UART.setCurrent(0.0f);
  commandedCurrentA = 0.0f;

  if (!applyAdvancedVescSetting(key, value)) {
    bleNotifyLine("ERR|ADV_VALUE");
    return true;
  }

  bleNotifyLine("ACK|ADV_SET|" + key);
  return true;
}

bool handleAndroidConfigPacket(const String &packet) {
  if (handleAdvancedAppPacket(packet)) return true;

  if (packet == "CFG|PING") {
    bleNotifyLine("ACK|PONG|V26");
    return true;
  }

  if (packet == "CFG|GET") {
    sendAppConfigState();
    return true;
  }

  if (packet == "CFG|RESTART") {
    if (!controllerSafeToStore()) {
      bleNotifyLine("ERR|STOP_FIRST");
      return true;
    }
    bleNotifyLine("ACK|RESTARTING");
    delay(40);
    ESP.restart();
    return true;
  }

  bool isApply = packet.startsWith("CFG|APPLY|");
  bool isSave = packet.startsWith("CFG|SAVE|");
  if (!isApply && !isSave) return false;

  if (!controllerSafeToStore()) {
    bleNotifyLine("ERR|STOP_FIRST");
    return true;
  }

  RemoteRideConfig cfg;
  if (!parseRemoteRideConfig(packet, cfg)) {
    bleNotifyLine("ERR|INVALID_VALUE");
    return true;
  }

  activateRemoteRideConfig(cfg);

  // Keep the scooter stopped while controller configuration packets are sent.
  UART.setCurrent(0.0f);
  commandedCurrentA = 0.0f;

  if (isSave) {
    requestSaveAndRestart(millis());
    if (saveRestartInProgress) {
      bleNotifyLine("ACK|SAVE|RESTARTING");
    } else {
      bleNotifyLine("ERR|SAVE_BLOCKED");
    }
  } else {
    applyControllerSettingsToVesc(false);
    bleNotifyLine("ACK|APPLY");
    sendAppConfigState();
  }

  return true;
}

void parseNavigationPacket(const String &packet) {
  if (handleAndroidConfigPacket(packet)) return;
  if (packet == "CLR") {
    navState.active = false;
    navState.routeCount = 0;
    navRoadCount = 0;
    navMapDirty = true;
    navMapLastGeometryRxMs = millis();
    navState.lastUpdateMs = millis();
    return;
  }

  if (packet == "ROADCLR") {
    navRoadCount = 0;
    navMapDirty = true;
    navMapLastGeometryRxMs = millis();
    navState.lastUpdateMs = millis();
    return;
  }

  if (packet.startsWith("TIME|")) {
    copyNavText(phoneTime, sizeof(phoneTime), navField(packet, 1));
    lastPhoneTimeMs = millis();
    return;
  }

  if (packet.startsWith("NAV|")) {
    copyNavText(navState.turn, sizeof(navState.turn), navField(packet, 1));
    long distanceValue = navField(packet, 2).toInt();
    navState.distanceM = (distanceValue > 0L) ? (int)distanceValue : 0;
    copyNavText(navState.road, sizeof(navState.road), navField(packet, 3));
    navState.bearingDeg = navField(packet, 4).toInt();
    long etaValue = navField(packet, 5).toInt();
    navState.etaMin = (etaValue > 0L) ? (int)etaValue : 0;
    long remainingValue = navField(packet, 6).toInt();
    navState.remainingM = (remainingValue > 0L) ? (int)remainingValue : 0;
    String arrivalValue = navField(packet, 7);
    if (arrivalValue.length() >= 4) {
      copyNavText(navState.arrival, sizeof(navState.arrival), arrivalValue);
    }
    navState.active = true;
    navState.lastUpdateMs = millis();
    return;
  }

  if (packet.startsWith("MAP|")) {
    String points = packet.substring(4);
    uint8_t count = 0;
    int start = 0;

    while (start < points.length() && count < NAV_MAX_ROUTE_POINTS) {
      int semi = points.indexOf(';', start);
      if (semi < 0) semi = points.length();

      String pair = points.substring(start, semi);
      int comma = pair.indexOf(',');
      if (comma > 0) {
        int x = constrain(pair.substring(0, comma).toInt(), 0, 100);
        int y = constrain(pair.substring(comma + 1).toInt(), 0, 100);
        navState.route[count].x = (int8_t)x;
        navState.route[count].y = (int8_t)y;
        count++;
      }
      start = semi + 1;
    }

    navState.routeCount = count;
    navMapDirty = true;
    navMapLastGeometryRxMs = millis();
    navState.lastUpdateMs = millis();
    return;
  }

  if (packet.startsWith("ROAD|") && navRoadCount < NAV_MAX_ROADS) {
    int firstSep = packet.indexOf('|', 5);
    if (firstSep < 0) return;

    int weight = packet.substring(5, firstSep).toInt();
    if (weight < 1) weight = 1;
    if (weight > 2) weight = 2;

    String points = packet.substring(firstSep + 1);
    NavRoad &road = navRoads[navRoadCount];
    road.count = 0;
    road.weight = (uint8_t)weight;

    int start = 0;
    while (start < points.length() && road.count < NAV_MAX_ROAD_POINTS) {
      int semi = points.indexOf(';', start);
      if (semi < 0) semi = points.length();

      String pair = points.substring(start, semi);
      int comma = pair.indexOf(',');
      if (comma > 0) {
        long xRaw = pair.substring(0, comma).toInt();
        long yRaw = pair.substring(comma + 1).toInt();

        int x = (int)xRaw;
        int y = (int)yRaw;
        if (x >= 0 && x <= 100 && y >= 0 && y <= 100) {
          road.point[road.count].x = (int8_t)x;
          road.point[road.count].y = (int8_t)y;
          road.count++;
        }
      }
      start = semi + 1;
    }

    if (road.count >= 2) {
      navRoadCount++;
      navMapDirty = true;
      navMapLastGeometryRxMs = millis();
      navState.lastUpdateMs = millis();
    }
    return;
  }
}

void serviceNavigationPackets() {
  if (navRxQueue == nullptr) return;

  BleRxPacket rx;
  uint8_t processed = 0;
  unsigned long budgetStartUs = micros();

  while (processed < BLE_PACKETS_PER_LOOP &&
         (unsigned long)(micros() - budgetStartUs) < BLE_PARSE_BUDGET_US &&
         xQueueReceive(navRxQueue, &rx, 0) == pdTRUE) {
    processed++;

    String packet(rx.data);
    int start = 0;
    while (start < packet.length()) {
      int nl = packet.indexOf('\n', start);
      if (nl < 0) nl = packet.length();
      String line = packet.substring(start, nl);
      line.trim();
      if (line.length() > 0) parseNavigationPacket(line);
      start = nl + 1;
    }
  }
}


void drawNavigationTurnIcon(int cx, int cy, const char *turn, uint16_t color) {
  // Simple high-contrast maneuver icons.
  if (strcmp(turn, "L") == 0 || strcmp(turn, "SL") == 0) {
    tft.drawFastHLine(cx - 13, cy, 21, color);
    tft.drawFastVLine(cx + 7, cy, 15, color);
    tft.fillTriangle(cx - 13, cy, cx - 3, cy - 8, cx - 3, cy + 8, color);
  }
  else if (strcmp(turn, "R") == 0 || strcmp(turn, "SR") == 0) {
    tft.drawFastHLine(cx - 8, cy, 21, color);
    tft.drawFastVLine(cx - 8, cy, 15, color);
    tft.fillTriangle(cx + 13, cy, cx + 3, cy - 8, cx + 3, cy + 8, color);
  }
  else if (strcmp(turn, "U") == 0) {
    tft.drawCircle(cx, cy + 2, 10, color);
    tft.fillRect(cx + 8, cy + 2, 5, 12, TFT_BLACK);
    tft.fillTriangle(cx - 11, cy - 5, cx - 2, cy - 11, cx - 2, cy + 1, color);
  }
  else if (strcmp(turn, "ARRIVE") == 0) {
    tft.drawCircle(cx, cy, 11, color);
    tft.fillCircle(cx, cy, 4, color);
  }
  else {
    tft.drawFastVLine(cx, cy - 13, 26, color);
    tft.fillTriangle(cx, cy - 15, cx - 8, cy - 5, cx + 8, cy - 5, color);
  }
}

String compactNavDistance(int meters) {
  if (meters >= 1000) {
    return String((float)meters / 1000.0f, 1) + " km";
  }
  return String(meters) + " m";
}

void drawNavigationRouteMap(int x, int y, int w, int h) {
  uint16_t mapBg = tft.color565(9, 15, 18);
  uint16_t localRoad = tft.color565(78, 88, 92);
  uint16_t majorRoad = tft.color565(135, 145, 148);
  uint16_t roadEdge = tft.color565(35, 43, 47);
  uint16_t routeColor = uiAccent();

  tft.fillRoundRect(x, y, w, h, 5, mapBg);

  // Draw real nearby OpenStreetMap road geometry sent by the Android page.
  for (uint8_t r = 0; r < navRoadCount; r++) {
    NavRoad &road = navRoads[r];
    if (road.count < 2) continue;

    uint16_t color = (road.weight >= 2) ? majorRoad : localRoad;

    for (uint8_t i = 1; i < road.count; i++) {
      int x1 = x + 2 + ((int)road.point[i - 1].x * (w - 4)) / 100;
      int y1 = y + 2 + ((int)road.point[i - 1].y * (h - 4)) / 100;
      int x2 = x + 2 + ((int)road.point[i].x * (w - 4)) / 100;
      int y2 = y + 2 + ((int)road.point[i].y * (h - 4)) / 100;

      if (road.weight >= 2) {
        // Dark edge + brighter centre makes major roads easier to read.
        tft.drawLine(x1 - 1, y1, x2 - 1, y2, roadEdge);
        tft.drawLine(x1 + 1, y1, x2 + 1, y2, roadEdge);
      }
      tft.drawLine(x1, y1, x2, y2, color);
    }
  }

  // Highlight the chosen route over the surrounding road network.
  if (navState.routeCount >= 2) {
    for (uint8_t i = 1; i < navState.routeCount; i++) {
      int x1 = x + 2 + ((int)navState.route[i - 1].x * (w - 4)) / 100;
      int y1 = y + 2 + ((int)navState.route[i - 1].y * (h - 4)) / 100;
      int x2 = x + 2 + ((int)navState.route[i].x * (w - 4)) / 100;
      int y2 = y + 2 + ((int)navState.route[i].y * (h - 4)) / 100;

      tft.drawLine(x1, y1, x2, y2, routeColor);
      tft.drawLine(x1 + 1, y1, x2 + 1, y2, routeColor);
    }
  }

  // Rider is intentionally lower than centre: more of the map is visible ahead.
  int px = x + w / 2;
  int py = y + (h * 72) / 100;
  tft.fillCircle(px, py, 3, TFT_WHITE);
  tft.drawCircle(px, py, 5, routeColor);

  // Heading marker: map data from the phone is rotated so "up" = direction of travel.
  tft.fillTriangle(px, py - 10, px - 3, py - 5, px + 3, py - 5, TFT_WHITE);

  tft.setTextFont(1);
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(tft.color565(145, 155, 158), mapBg);
  tft.drawString("OSM", x + w - 3, y + h - 2);
  tft.setTextDatum(TL_DATUM);
}





// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

int pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WDITH];

  png.getLineAsRGB565(
    pDraw,
    lineBuffer,
    PNG_RGB565_BIG_ENDIAN,
    0xffffffff
  );

  tft.pushImage(
    xpos,
    ypos + pDraw->y,
    pDraw->iWidth,
    1,
    lineBuffer
  );

  return 1;
}


uint16_t uiCardColor() { return tft.color565(18, 29, 36); }
uint16_t uiCardBorder() { return tft.color565(55, 75, 86); }
uint16_t uiAccent() { return tft.color565(0, 210, 185); }
uint16_t uiOrange() { return tft.color565(255, 165, 45); }

// -----------------------------------------------------------------------------
// DIRECT VESC CONFIGURATION VIA LISPBM REPL
// -----------------------------------------------------------------------------
// V15 no longer depends on event-data-rx or a companion helper file.
//
// VESC firmware exposes COMM_LISP_REPL_CMD. We send a Lisp expression directly
// to the controller, so the controller itself executes conf-set/conf-store.
//
// Each conf-set is wrapped in trap. This is important because
// foc-fw-current-max only exists on newer firmware; an unsupported FW parameter
// must not prevent Drive Current / Regen from being stored.
#define VESC_COMM_LISP_REPL_CMD 138

uint16_t vescPacketCrc16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)buf[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void sendVescPayload(const uint8_t *payload, uint16_t len) {
  if (len == 0) return;

  uint16_t crc = vescPacketCrc16(payload, len);

  if (len <= 255) {
    // Standard short VESC packet.
    VescSerial.write((uint8_t)2);
    VescSerial.write((uint8_t)len);
  } else {
    // Long VESC packet: start byte 3 followed by a 16-bit payload length.
    VescSerial.write((uint8_t)3);
    VescSerial.write((uint8_t)(len >> 8));
    VescSerial.write((uint8_t)(len & 0xFF));
  }

  VescSerial.write(payload, len);
  VescSerial.write((uint8_t)(crc >> 8));
  VescSerial.write((uint8_t)(crc & 0xFF));
  VescSerial.write((uint8_t)3);
}

void sendVescLispExpression(const String &expr) {
  // A long VESC packet lets all related settings be applied in one Lisp REPL
  // expression. This matters because VESC rate-limits REPL commands.
  size_t exprLen = expr.length();
  if (exprLen == 0 || exprLen > 500) {
#ifdef DEBUG_MODE
    Serial.println("Lisp expression too long - not sent.");
#endif
    return;
  }

  static uint8_t payload[512];
  payload[0] = VESC_COMM_LISP_REPL_CMD;
  memcpy(payload + 1, expr.c_str(), exprLen);
  payload[1 + exprLen] = 0;

  sendVescPayload(payload, (uint16_t)(exprLen + 2));
}

String vescNumber(float value, unsigned int decimals = 1) {
  // ESP32 Arduino Core 3.x has both float and double String constructors.
  // Using unsigned int removes the overload ambiguity.
  return String(value, decimals);
}

void applyControllerSettingsToVesc(bool storeToFlash) {
  unsigned long now = millis();
  if (lastVescLispCommandMs != 0 &&
      (now - lastVescLispCommandMs) < VESC_LISP_MIN_INTERVAL_MS) {
    // VESC ignores REPL commands sent too close together. Keep one pending
    // request and send it from updateTelemetry once the interval has elapsed.
    maxCurrentApplyPending = true;
    fieldWeakeningApplyPending = true;
    if (storeToFlash) vescStoreApplyPending = true;
    return;
  }

  float currentA =
      constrain(userMaxDriveCurrentA, 5.0f, HARD_MAX_DRIVE_CURRENT_A);
  float fwA =
      constrain(userFieldWeakeningA, 0.0f, HARD_MAX_FIELD_WEAKENING_A);
  float regenA =
      constrain(userRegenBrakeA, 0.0f, HARD_MAX_REGEN_BRAKE_A);
  float batteryMaxA =
      constrain(userBatteryCurrentMaxA,
                MIN_BATTERY_CURRENT_MAX_A,
                HARD_MAX_BATTERY_CURRENT_MAX_A);
  float batteryRegenA =
      constrain(userBatteryRegenMaxA, 0.0f, HARD_MAX_BATTERY_REGEN_A);
  float cutoffStartV =
      constrain(userBatteryCutoffStartV,
                MIN_BATTERY_CUTOFF_START_V,
                MAX_BATTERY_CUTOFF_START_V);
  float cutoffEndV =
      constrain(userBatteryCutoffEndV,
                MIN_BATTERY_CUTOFF_END_V,
                MAX_BATTERY_CUTOFF_END_V);

  // One atomic expression avoids the VESC Lisp REPL 0.5-second command gate.
  String cmd = "(progn ";
  cmd += "(trap (conf-set 'l-current-max " + vescNumber(currentA) + ")) ";
  cmd += "(trap (conf-set 'foc-fw-current-max " + vescNumber(fwA) + ")) ";
  cmd += "(trap (conf-set 'l-current-min -" + vescNumber(regenA) + ")) ";
  cmd += "(trap (conf-set 'l-in-current-max " + vescNumber(batteryMaxA) + ")) ";
  cmd += "(trap (conf-set 'l-in-current-min -" + vescNumber(batteryRegenA) + ")) ";
  cmd += "(trap (conf-set 'l-battery-cut-start " + vescNumber(cutoffStartV) + ")) ";
  cmd += "(trap (conf-set 'l-battery-cut-end " + vescNumber(cutoffEndV) + ")) ";
  if (storeToFlash) cmd += "(conf-store) ";
  cmd += ")";

  sendVescLispExpression(cmd);
  lastVescLispCommandMs = now;
  maxCurrentApplyPending = false;
  fieldWeakeningApplyPending = false;
  vescStoreApplyPending = false;

#ifdef DEBUG_MODE
  Serial.print("Atomic VESC config, bytes=");
  Serial.print(cmd.length());
  Serial.print(" store=");
  Serial.println(storeToFlash ? "YES" : "NO");
#endif
}

void applyFieldWeakeningToVesc() {
  applyControllerSettingsToVesc(false);
}

void applyMaxCurrentToVesc() {
  applyControllerSettingsToVesc(false);
}

bool controllerSafeToStore() {
  bool freshTelemetry = vescSeen && ((millis() - lastVescRxMs) < 300);
  return freshTelemetry &&
         fabsf(speed) < 0.5f &&
         throttlePercent <= 0.001f &&
         commandedCurrentA <= 0.05f &&
         !eBrakeActive;
}

void saveDisplaySettings();

void requestSaveAndRestart(unsigned long now) {
  if (saveRestartInProgress) return;

  if (!controllerSafeToStore()) {
    saveBlockedMessage = true;
    saveBlockedUntilMs = now + 1600;
    settingsControlsDirty = true;
    return;
  }

  // Save ESP32 settings first, then send one direct Lisp REPL command that
  // changes the active VESC motor config and calls conf-store.
  saveDisplaySettings();

  // Stop commanding the motor before conf-store. VESC documents conf-store as
  // a motor-stopping operation.
  UART.setCurrent(0.0f);
  commandedCurrentA = 0.0f;

  applyControllerSettingsToVesc(true);

  settingsDirty = false;
  saveBlockedMessage = false;
  saveRestartInProgress = true;
  saveRestartStartedMs = now;
  settingsControlsDirty = true;
}

void saveDisplaySettings() {
  DisplaySettings s;
  s.magic = SETTINGS_MAGIC;
  s.maxCurrentA = userMaxDriveCurrentA;
  s.fieldWeakeningA = userFieldWeakeningA;
  s.regenBrakeA = userRegenBrakeA;
  s.batteryCurrentMaxA = userBatteryCurrentMaxA;
  s.batteryRegenMaxA = userBatteryRegenMaxA;
  s.policeLimitKmh = userPoliceLimitKmh;
  s.currentRiseAPerSec = userCurrentRiseAPerSec;
  s.batteryCutoffStartV = userBatteryCutoffStartV;
  s.batteryCutoffEndV = userBatteryCutoffEndV;
  s.police = policeMode ? 1 : 0;
  s.regenAbs = regenAbsEnabled ? 1 : 0;
  s.releaseEBrake = releaseEBrakeEnabled ? 1 : 0;
  s.reserved = 0;
  EEPROM_writeAnything(SETTINGS_EEPROM_ADDR, s);
  EEPROM.commit();
}


void loadDisplaySettings() {
  DisplaySettings s;
  EEPROM_readAnything(SETTINGS_EEPROM_ADDR, s);

  bool valid =
      s.magic == SETTINGS_MAGIC &&
      s.maxCurrentA >= 5.0f &&
      s.maxCurrentA <= HARD_MAX_DRIVE_CURRENT_A &&
      s.fieldWeakeningA >= 0.0f &&
      s.fieldWeakeningA <= HARD_MAX_FIELD_WEAKENING_A &&
      s.regenBrakeA >= 0.0f &&
      s.regenBrakeA <= HARD_MAX_REGEN_BRAKE_A &&
      s.batteryCurrentMaxA >= MIN_BATTERY_CURRENT_MAX_A &&
      s.batteryCurrentMaxA <= HARD_MAX_BATTERY_CURRENT_MAX_A &&
      s.batteryRegenMaxA >= 0.0f &&
      s.batteryRegenMaxA <= HARD_MAX_BATTERY_REGEN_A &&
      s.policeLimitKmh >= MIN_POLICE_LIMIT_KMH &&
      s.policeLimitKmh <= MAX_POLICE_LIMIT_KMH &&
      s.currentRiseAPerSec >= MIN_CURRENT_RISE_A_PER_SEC &&
      s.currentRiseAPerSec <= MAX_CURRENT_RISE_A_PER_SEC &&
      s.batteryCutoffStartV >= MIN_BATTERY_CUTOFF_START_V &&
      s.batteryCutoffStartV <= MAX_BATTERY_CUTOFF_START_V &&
      s.batteryCutoffEndV >= MIN_BATTERY_CUTOFF_END_V &&
      s.batteryCutoffEndV <= MAX_BATTERY_CUTOFF_END_V &&
      s.batteryCutoffEndV <= (s.batteryCutoffStartV - 1.0f);

  if (valid) {
    userMaxDriveCurrentA = s.maxCurrentA;
    userFieldWeakeningA = s.fieldWeakeningA;
    userRegenBrakeA = s.regenBrakeA;
    userBatteryCurrentMaxA = s.batteryCurrentMaxA;
    userBatteryRegenMaxA = s.batteryRegenMaxA;
    userPoliceLimitKmh = s.policeLimitKmh;
    userCurrentRiseAPerSec = s.currentRiseAPerSec;
    userBatteryCutoffStartV = s.batteryCutoffStartV;
    userBatteryCutoffEndV = s.batteryCutoffEndV;
    policeMode = (s.police != 0);
    regenAbsEnabled = (s.regenAbs != 0);
    releaseEBrakeEnabled = (s.releaseEBrake != 0);
  } else {
    userMaxDriveCurrentA = DEFAULT_MAX_DRIVE_CURRENT_A;
    userFieldWeakeningA = DEFAULT_FIELD_WEAKENING_A;
    userRegenBrakeA = DEFAULT_REGEN_BRAKE_A;
    userBatteryCurrentMaxA = DEFAULT_BATTERY_CURRENT_MAX_A;
    userBatteryRegenMaxA = DEFAULT_BATTERY_REGEN_A;
    userPoliceLimitKmh = DEFAULT_POLICE_LIMIT_KMH;
    userCurrentRiseAPerSec = DEFAULT_CURRENT_RISE_A_PER_SEC;
    userBatteryCutoffStartV = DEFAULT_BATTERY_CUTOFF_START_V;
    userBatteryCutoffEndV = DEFAULT_BATTERY_CUTOFF_END_V;
    policeMode = false;
    regenAbsEnabled = true;
    releaseEBrakeEnabled = true;
    saveDisplaySettings();
  }

  // Re-apply the display-saved VESC values in RAM after boot.
  maxCurrentApplyPending = true;
  fieldWeakeningApplyPending = true;
  vescStoreApplyPending = false;
  settingsDirty = false;
}


void restoreMainBackground() {
  tft.fillScreen(TFT_BLACK);

#ifdef DO_LOGO_DRAW
  // Restore the same background_image.h used by the original project UI.
  int16_t rc_mainbg =
      png.openFLASH((uint8_t *)background_image, sizeof(background_image), pngDraw);
  if (rc_mainbg == PNG_SUCCESS) {
    tft.startWrite();
    png.decode(NULL, 0);
    tft.endWrite();
  }
  png.close();
#endif

  classicDashboardNeedsRedraw = true;
  uiNeedsFullRedraw = true;
}

bool readTouchScreen(uint16_t &sx, uint16_t &sy) {
  if (!touch.touched()) return false;

  TS_Point p = touch.getPoint();
  int32_t ax = p.x;
  int32_t ay = p.y;

  // Convert raw touch coordinates to portrait-oriented 320x240 screen coords.
  int32_t mx = map(ax, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319);
  int32_t my = map(ay, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239);
  mx = constrain(mx, 0, 319);
  my = constrain(my, 0, 239);

#if TOUCH_SWAP_XY
  int32_t tmp = mx;
  mx = map(my, 0, 239, 0, 319);
  my = map(tmp, 0, 319, 0, 239);
#endif
#if TOUCH_INVERT_X
  mx = 319 - mx;
#endif
#if TOUCH_INVERT_Y
  my = 239 - my;
#endif

  sx = (uint16_t)mx;
  sy = (uint16_t)my;
  return true;
}

bool hit(uint16_t x, uint16_t y, int x1, int y1, int x2, int y2) {
  return x >= x1 && x <= x2 && y >= y1 && y <= y2;
}


void finishUiTransition(UiScreen target) {
  uiScreen = target;
  uiNeedsFullRedraw = true;
  uiTransition = TRANS_NONE;

  if (uiScreen == UI_MAIN) {
    restoreMainBackground();
  } else {
    tft.fillScreen(tft.color565(7, 12, 16));
  }
}


void startUiTransition(UiScreen target, UiTransitionType type) {
  if (uiTransition != TRANS_NONE) return;

  uiTransitionTarget = target;
  uiTransition = type;
  uiTransitionStartMs = millis();
  uiTransitionLastFrameMs = 0;

  // Slider drag state should never survive a page transition.
  activeSettingsSlider = 0;
}


void startSettingsScroll(uint8_t targetSection, UiTransitionType type) {
  if (uiTransition != TRANS_NONE || uiScreen != UI_SETTINGS) return;
  if (targetSection > 1 || targetSection == settingsSection) return;

  settingsTransitionTarget = targetSection;
  uiTransition = type;
  uiTransitionStartMs = millis();
  uiTransitionLastFrameMs = 0;
  activeSettingsSlider = 0;
}

void setUiScreen(UiScreen s) {
  if (s == uiScreen && uiTransition == TRANS_NONE) return;

  if (uiScreen == UI_MAIN && s != UI_MAIN) {
    startUiTransition(s, TRANS_OPEN_MENU);
  } else if (s == UI_MAIN) {
    startUiTransition(s, TRANS_CLOSE_MENU);
  } else {
    startUiTransition(s, TRANS_PAGE_NEXT);
  }
}

UiScreen pageAfter(UiScreen s) {
  if (s == UI_CONTROLLER) return UI_TRIP;
  if (s == UI_TRIP) return UI_BATTERY;
  if (s == UI_BATTERY) return UI_SETTINGS;
  return UI_CONTROLLER;
}

UiScreen pageBefore(UiScreen s) {
  if (s == UI_CONTROLLER) return UI_SETTINGS;
  if (s == UI_TRIP) return UI_CONTROLLER;
  if (s == UI_BATTERY) return UI_TRIP;
  return UI_BATTERY;
}

void nextMenuPage() {
  if (uiScreen == UI_MAIN || uiTransition != TRANS_NONE) return;
  startUiTransition(pageAfter(uiScreen), TRANS_PAGE_NEXT);
}

void previousMenuPage() {
  if (uiScreen == UI_MAIN || uiTransition != TRANS_NONE) return;
  startUiTransition(pageBefore(uiScreen), TRANS_PAGE_PREV);
}

float easeOutCubic(float t) {
  t = constrain(t, 0.0f, 1.0f);
  float p = 1.0f - t;
  return 1.0f - p * p * p;
}

void updateUiTransition(unsigned long now) {
  if (uiTransition == TRANS_NONE) return;

  if (uiTransitionLastFrameMs != 0 &&
      (now - uiTransitionLastFrameMs) < UI_TRANSITION_FRAME_MS) {
    return;
  }
  uiTransitionLastFrameMs = now;

  float rawProgress =
      (float)(now - uiTransitionStartMs) / (float)UI_TRANSITION_MS;
  float p = easeOutCubic(rawProgress);
  uint16_t bg = tft.color565(7, 12, 16);

  if (uiTransition == TRANS_OPEN_MENU) {
    int startW = 66;
    int startH = 34;
    int startX = 127;
    int startY = 173;
    int w = startW + (int)((320 - startW) * p);
    int h = startH + (int)((240 - startH) * p);
    int x = (int)(startX * (1.0f - p));
    int y = (int)(startY * (1.0f - p));
    tft.fillRoundRect(x, y, w, h, 9, bg);
    tft.drawRoundRect(x, y, w, h, 9, uiAccent());
  }
  else if (uiTransition == TRANS_PAGE_NEXT) {
    int coverW = (int)(320.0f * p);
    if (coverW > 0) {
      int x = 320 - coverW;
      tft.fillRect(x, 0, coverW, 240, bg);
      if (x > 1) tft.drawFastVLine(x - 1, 0, 240, uiAccent());
    }
  }
  else if (uiTransition == TRANS_PAGE_PREV) {
    int coverW = (int)(320.0f * p);
    if (coverW > 0) {
      tft.fillRect(0, 0, coverW, 240, bg);
      if (coverW < 319) tft.drawFastVLine(coverW, 0, 240, uiAccent());
    }
  }
  else if (uiTransition == TRANS_CLOSE_MENU) {
    int edge = (int)(160.0f * p);
    if (edge > 0) {
      tft.fillRect(0, 0, edge, 240, TFT_BLACK);
      tft.fillRect(320 - edge, 0, edge, 240, TFT_BLACK);
    }
  }
  else if (uiTransition == TRANS_SETTINGS_UP ||
           uiTransition == TRANS_SETTINGS_DOWN) {
    const int top = 31;
    const int height = 164;
    int cover = (int)(height * p);

    if (uiTransition == TRANS_SETTINGS_UP) {
      int y = top + height - cover;
      tft.fillRect(0, y, 320, cover, bg);
      if (y > top) tft.drawFastHLine(0, y - 1, 320, uiAccent());
    } else {
      tft.fillRect(0, top, 320, cover, bg);
      if (top + cover < top + height) {
        tft.drawFastHLine(0, top + cover, 320, uiAccent());
      }
    }
  }

  if (rawProgress >= 1.0f) {
    if (uiTransition == TRANS_SETTINGS_UP ||
        uiTransition == TRANS_SETTINGS_DOWN) {
      settingsSection = settingsTransitionTarget;
      uiTransition = TRANS_NONE;
      uiNeedsFullRedraw = true;
      settingsControlsDirty = true;
    } else {
      finishUiTransition(uiTransitionTarget);
    }
  }
}


void drawArrowButton(int x, int y, bool right) {
  // Smaller navigation buttons for the compact menu.
  tft.fillRoundRect(x, y, 30, 30, 6, uiCardColor());
  tft.drawRoundRect(x, y, 30, 30, 6, uiCardBorder());
  uint16_t c = TFT_WHITE;
  if (right) {
    tft.fillTriangle(x + 10, y + 7, x + 10, y + 23, x + 22, y + 15, c);
  } else {
    tft.fillTriangle(x + 20, y + 7, x + 20, y + 23, x + 8, y + 15, c);
  }
}



void drawMenuChrome(const char *title, int pageIndex) {
  uint16_t bg = tft.color565(7, 12, 16);
  tft.fillScreen(bg);

  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(DATAFONTSMALL2);
  tft.setTextColor(TFT_WHITE, bg);
  tft.drawString(title, 12, 4);

  tft.fillCircle(242, 14, 3, vescSeen ? TFT_GREEN : TFT_RED);
  tft.setTextFont(1);
  tft.setTextColor(tft.color565(130, 150, 160), bg);
  tft.drawString(vescSeen ? "LIVE" : "OFF", 250, 10);

  tft.drawFastHLine(12, 30, 250, uiCardBorder());
  tft.drawFastHLine(12, 30, 42, uiAccent());

  tft.fillRoundRect(284, 4, 30, 24, 6, uiCardColor());
  tft.drawRoundRect(284, 4, 30, 24, 6, uiCardBorder());
  tft.drawLine(293, 11, 305, 22, TFT_WHITE);
  tft.drawLine(305, 11, 293, 22, TFT_WHITE);

  drawArrowButton(8, 202, false);
  drawArrowButton(274, 202, true);

  for (int i = 0; i < 4; i++) {
    uint16_t c = (i == pageIndex) ? uiAccent() : uiCardBorder();
    tft.fillCircle(143 + i * 11, 218, 2, c);
  }

  tft.setTextFont(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(tft.color565(110, 130, 140), bg);
  tft.drawString(String(pageIndex + 1) + "/4", 160, 202);
  tft.setTextDatum(TL_DATUM);
}


void drawCardFrame(int x, int y, int w, int h, const char *label) {
  tft.fillRoundRect(x, y, w, h, 6, uiCardColor());
  tft.drawRoundRect(x, y, w, h, 6, uiCardBorder());

  // Tiny accent detail inspired by the reference display.
  tft.drawFastHLine(x + 6, y + 3, 14, uiAccent());

  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(1);
  tft.setTextColor(tft.color565(155, 175, 185), uiCardColor());
  tft.drawString(label, x + 6, y + 7);
}

void updateCardValue(int x, int y, int w, int h,
                     const String &value, uint16_t valueColor) {
  int vx = x + 4;
  int vy = y + 20;
  int vw = w - 8;
  int vh = h - 24;
  tft.fillRect(vx, vy, vw, vh, uiCardColor());

  // Jersey numerals are cleaner and more dashboard-like than the tiny label
  // font. Centering also prevents values from looking uneven between cards.
  tft.setTextDatum(MC_DATUM);
  tft.setFreeFont(DATAFONTSMALL2);
  tft.setTextColor(valueColor, uiCardColor());
  tft.drawString(value, x + w / 2, y + 32);
  tft.setTextDatum(TL_DATUM);
}


int estimatedBatteryPercent(float volts) {
  int pct = (int)((volts - 30.0f) * 100.0f / (42.0f - 30.0f));
  return constrain(pct, 0, 100);
}

// Compact card geometry shared by controller/trip/battery pages.
#define CARD_X1 14
#define CARD_X2 113
#define CARD_X3 212
#define CARD_W 94
#define CARD_H 48
#define CARD_Y1 40
#define CARD_Y2 96

void updateControllerValues() {
  updateCardValue(CARD_X1, CARD_Y1, CARD_W, CARD_H,
                  String(UART.data.tempMotor, 0) + " C", TFT_CYAN);
  updateCardValue(CARD_X2, CARD_Y1, CARD_W, CARD_H,
                  String(UART.data.tempMosfet, 0) + " C", TFT_GREEN);
  updateCardValue(CARD_X3, CARD_Y1, CARD_W, CARD_H,
                  String(UART.data.inpVoltage, 1) + " V", uiOrange());

  updateCardValue(CARD_X1, CARD_Y2, CARD_W, CARD_H,
                  String(UART.data.avgInputCurrent, 1) + " A", TFT_WHITE);
  updateCardValue(CARD_X2, CARD_Y2, CARD_W, CARD_H,
                  String(UART.data.avgMotorCurrent, 1) + " A", TFT_WHITE);
  updateCardValue(CARD_X3, CARD_Y2, CARD_W, CARD_H,
                  String(throttleMv) + " mV", uiAccent());
}

void drawControllerPage() {
  drawMenuChrome("Controller", 0);

  drawCardFrame(CARD_X1, CARD_Y1, CARD_W, CARD_H, "Motor temp");
  drawCardFrame(CARD_X2, CARD_Y1, CARD_W, CARD_H, "VESC temp");
  drawCardFrame(CARD_X3, CARD_Y1, CARD_W, CARD_H, "Bus voltage");

  drawCardFrame(CARD_X1, CARD_Y2, CARD_W, CARD_H, "Battery A");
  drawCardFrame(CARD_X2, CARD_Y2, CARD_W, CARD_H, "Motor A");
  drawCardFrame(CARD_X3, CARD_Y2, CARD_W, CARD_H, "Throttle");

  updateControllerValues();
}

void updateTripValues() {
  unsigned long movingMs =
      movingAccumMs + (movingNow ? (millis() - movingStartMs) : 0);
  float movingHours = movingMs / 3600000.0f;

  averageSpeedKmh =
      movingHours > 0.001f ? sessionTripKm / movingHours : 0.0f;

  if (!isfinite(averageSpeedKmh) || averageSpeedKmh < 0.0f) {
    averageSpeedKmh = 0.0f;
  }
  // A second sanity guard protects the UI from one corrupted telemetry sample.
  float plausibleMax = max(5.0f, sessionMaxSpeed + 3.0f);
  averageSpeedKmh = min(averageSpeedKmh, plausibleMax);
  averageSpeedKmh = min(averageSpeedKmh, MAX_VALID_SPEED_KMH);

  updateCardValue(CARD_X1, CARD_Y1, CARD_W, CARD_H,
                  String(total_km, 1) + " km", TFT_WHITE);
  updateCardValue(CARD_X2, CARD_Y1, CARD_W, CARD_H,
                  String(sessionTripKm, 2) + " km", uiAccent());
  updateCardValue(CARD_X3, CARD_Y1, CARD_W, CARD_H,
                  String(sessionMaxSpeed, 1) + " km/h", uiOrange());

  updateCardValue(CARD_X1, CARD_Y2, CARD_W, CARD_H,
                  String(movingMs / 60000UL) + " min", TFT_WHITE);
  updateCardValue(CARD_X2, CARD_Y2, CARD_W, CARD_H,
                  String(averageSpeedKmh, 1) + " km/h", TFT_GREEN);
  updateCardValue(CARD_X3, CARD_Y2, CARD_W, CARD_H,
                  String(watts, 0) + " W", TFT_CYAN);
}


void drawTripPage() {
  drawMenuChrome("Ride stats", 1);

  drawCardFrame(CARD_X1, CARD_Y1, CARD_W, CARD_H, "Odometer");
  drawCardFrame(CARD_X2, CARD_Y1, CARD_W, CARD_H, "Trip");
  drawCardFrame(CARD_X3, CARD_Y1, CARD_W, CARD_H, "Max speed");

  drawCardFrame(CARD_X1, CARD_Y2, CARD_W, CARD_H, "Ride time");
  drawCardFrame(CARD_X2, CARD_Y2, CARD_W, CARD_H, "Avg speed");
  drawCardFrame(CARD_X3, CARD_Y2, CARD_W, CARD_H, "Power");

  updateTripValues();
}

void updateBatteryValues() {
  int pct = estimatedBatteryPercent(UART.data.inpVoltage);

  updateCardValue(CARD_X1, CARD_Y1, CARD_W, CARD_H,
                  String(pct) + " %", uiAccent());
  updateCardValue(CARD_X2, CARD_Y1, CARD_W, CARD_H,
                  String(UART.data.inpVoltage, 1) + " V", uiOrange());
  updateCardValue(CARD_X3, CARD_Y1, CARD_W, CARD_H,
                  String(UART.data.avgInputCurrent, 1) + " A", TFT_WHITE);

  updateCardValue(CARD_X1, CARD_Y2, CARD_W, CARD_H,
                  String(watts, 0) + " W", TFT_CYAN);
  updateCardValue(CARD_X2, CARD_Y2, CARD_W, CARD_H,
                  vescSeen ? "ONLINE" : "NO DATA",
                  vescSeen ? TFT_GREEN : TFT_RED);
  updateCardValue(CARD_X3, CARD_Y2, CARD_W, CARD_H,
                  String((int)UART.data.error),
                  UART.data.error == 0 ? TFT_GREEN : TFT_RED);
}

void drawBatteryPage() {
  drawMenuChrome("Battery", 2);

  drawCardFrame(CARD_X1, CARD_Y1, CARD_W, CARD_H, "Charge");
  drawCardFrame(CARD_X2, CARD_Y1, CARD_W, CARD_H, "Voltage");
  drawCardFrame(CARD_X3, CARD_Y1, CARD_W, CARD_H, "Current");

  drawCardFrame(CARD_X1, CARD_Y2, CARD_W, CARD_H, "Power");
  drawCardFrame(CARD_X2, CARD_Y2, CARD_W, CARD_H, "VESC");
  drawCardFrame(CARD_X3, CARD_Y2, CARD_W, CARD_H, "Fault");

  updateBatteryValues();
}

void drawCompactSlider(int y, float value, float minV, float maxV,
                       uint16_t accentColor) {
  const int sx1 = 82;
  const int sx2 = 296;
  const int sy = y;

  tft.fillRect(74, sy - 9, 228, 19, uiCardColor());
  tft.drawFastHLine(sx1, sy, sx2 - sx1, tft.color565(75, 92, 102));

  float ratio = (value - minV) / (maxV - minV);
  ratio = constrain(ratio, 0.0f, 1.0f);
  int knob = sx1 + (int)((sx2 - sx1) * ratio);

  tft.drawFastHLine(sx1, sy, max(0, knob - sx1), accentColor);
  tft.fillCircle(knob, sy, 6, TFT_WHITE);
}


void drawSettingsScrollIndicator() {
  uint16_t track = tft.color565(55, 70, 78);
  tft.fillRoundRect(312, 43, 4, 136, 2, track);
  int thumbY = settingsSection == 0 ? 45 : 121;
  tft.fillRoundRect(310, thumbY, 8, 56, 4, uiAccent());
}

void drawSettingsStatus() {
  uint16_t bg = tft.color565(7, 12, 16);
  tft.fillRect(182, 5, 94, 22, bg);
  tft.setTextDatum(TR_DATUM);
  tft.setTextFont(1);

  if (saveRestartInProgress) {
    tft.setTextColor(uiAccent(), bg);
    tft.drawString("SAVING...", 274, 11);
  } else if (saveBlockedMessage && millis() < saveBlockedUntilMs) {
    tft.setTextColor(TFT_RED, bg);
    tft.drawString("STOP FIRST", 274, 11);
  } else if (settingsDirty) {
    tft.setTextColor(uiOrange(), bg);
    tft.drawString("UNSAVED", 274, 11);
  } else {
    tft.setTextColor(TFT_GREEN, bg);
    tft.drawString("SAVED", 274, 11);
  }
  tft.setTextDatum(TL_DATUM);
}

void drawSettingsDynamic() {
  drawSettingsStatus();

  if (settingsSection == 0) {
    // Police mode toggle.
    tft.fillRect(244, 42, 52, 30, uiCardColor());
    uint16_t toggleColor =
        policeMode ? uiAccent() : tft.color565(80, 85, 90);
    tft.fillRoundRect(249, 47, 42, 20, 10, toggleColor);
    int knobX = policeMode ? 281 : 259;
    tft.fillCircle(knobX, 57, 8, TFT_WHITE);

    tft.setTextFont(1);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(policeMode ? uiAccent() : tft.color565(130, 145, 150),
                     uiCardColor());
    tft.drawString(policeMode ? "ON" : "OFF", 240, 53);
    tft.setTextDatum(TL_DATUM);

    // Drive current value + slider.
    tft.fillRect(23, 89, 78, 19, uiCardColor());
    tft.setTextDatum(TR_DATUM);
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(uiAccent(), uiCardColor());
    tft.drawString(String(userMaxDriveCurrentA, 0) + " A", 98, 90);
    tft.setTextDatum(TL_DATUM);
    drawCompactSlider(118, userMaxDriveCurrentA, 5.0f,
                      HARD_MAX_DRIVE_CURRENT_A, uiAccent());

    // Field weakening value + slider.
    tft.fillRect(23, 144, 78, 19, uiCardColor());
    tft.setTextDatum(TR_DATUM);
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(uiOrange(), uiCardColor());
    tft.drawString(String(userFieldWeakeningA, 1) + " A", 98, 145);
    tft.setTextDatum(TL_DATUM);
    drawCompactSlider(173, userFieldWeakeningA, 0.0f,
                      HARD_MAX_FIELD_WEAKENING_A, uiOrange());
  } else {
    // Regen value + slider.
    tft.fillRect(23, 48, 78, 19, uiCardColor());
    tft.setTextDatum(TR_DATUM);
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_CYAN, uiCardColor());
    tft.drawString(String(userRegenBrakeA, 1) + " A", 98, 49);
    tft.setTextDatum(TL_DATUM);
    drawCompactSlider(78, userRegenBrakeA, 0.0f,
                      HARD_MAX_REGEN_BRAKE_A, TFT_CYAN);

    // Regen ABS toggle.
    tft.fillRect(244, 101, 52, 30, uiCardColor());
    uint16_t toggleColor =
        regenAbsEnabled ? uiAccent() : tft.color565(80, 85, 90);
    tft.fillRoundRect(249, 106, 42, 20, 10, toggleColor);
    int knobX = regenAbsEnabled ? 281 : 259;
    tft.fillCircle(knobX, 116, 8, TFT_WHITE);

    tft.setTextFont(1);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(regenAbsEnabled ? uiAccent() : tft.color565(130,145,150),
                     uiCardColor());
    tft.drawString(regenAbsEnabled ? "ON" : "OFF", 240, 112);
    tft.setTextDatum(TL_DATUM);

    // Save button state.
    uint16_t buttonColor = saveRestartInProgress
                               ? tft.color565(28, 90, 85)
                               : (controllerSafeToStore()
                                      ? uiAccent()
                                      : tft.color565(75, 80, 84));
    tft.fillRoundRect(35, 148, 250, 36, 8, buttonColor);
    tft.drawRoundRect(35, 148, 250, 36, 8,
                      saveRestartInProgress ? uiAccent() : uiCardBorder());
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, buttonColor);
    const char *buttonText = saveRestartInProgress
                                 ? "SAVING & RESTARTING"
                                 : (controllerSafeToStore()
                                        ? "SAVE & RESTART"
                                        : "STOP TO SAVE");
    tft.drawString(buttonText, 160, 166);
    tft.setTextDatum(TL_DATUM);
  }

  drawSettingsScrollIndicator();
  settingsControlsDirty = false;
}

void drawSettingsPage() {
  drawMenuChrome("Ride settings", 3);

  // Compact section label in the title bar. It stays clear of the first row.
  uint16_t menuBg = tft.color565(7, 12, 16);
  tft.setTextFont(1);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(uiAccent(), menuBg);
  tft.drawString(settingsSection == 0 ? "PERF 1/2" : "BRAKE 2/2", 128, 10);

  if (settingsSection == 0) {
    tft.fillRoundRect(14, 38, 292, 38, 6, uiCardColor());
    tft.drawRoundRect(14, 38, 292, 38, 6, uiCardBorder());
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, uiCardColor());
    tft.drawString("Police mode", 24, 45);
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(150, 170, 180), uiCardColor());
    tft.drawString("20 km/h soft cap", 24, 61);

    tft.fillRoundRect(14, 82, 292, 48, 6, uiCardColor());
    tft.drawRoundRect(14, 82, 292, 48, 6, uiCardBorder());
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, uiCardColor());
    tft.drawString("Drive current", 24, 89);
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(145,165,175), uiCardColor());
    tft.drawString("5 A", 81, 124);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String((int)HARD_MAX_DRIVE_CURRENT_A) + " A", 299, 124);
    tft.setTextDatum(TL_DATUM);

    tft.fillRoundRect(14, 137, 292, 48, 6, uiCardColor());
    tft.drawRoundRect(14, 137, 292, 48, 6, uiCardBorder());
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, uiCardColor());
    tft.drawString("Field weakening", 24, 144);
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(145,165,175), uiCardColor());
    tft.drawString("0 A", 81, 179);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String((int)HARD_MAX_FIELD_WEAKENING_A) + " A", 299, 179);
    tft.setTextDatum(TL_DATUM);

    tft.setTextFont(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(tft.color565(120, 145, 155), tft.color565(7,12,16));
    tft.drawString("Swipe up for braking settings", 160, 191);
    tft.setTextDatum(TL_DATUM);
  } else {
    tft.fillRoundRect(14, 38, 292, 52, 6, uiCardColor());
    tft.drawRoundRect(14, 38, 292, 52, 6, uiCardBorder());
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, uiCardColor());
    tft.drawString("Release regen", 24, 46);
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(145,165,175), uiCardColor());
    tft.drawString("0 A", 81, 84);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String((int)HARD_MAX_REGEN_BRAKE_A) + " A", 299, 84);
    tft.setTextDatum(TL_DATUM);

    tft.fillRoundRect(14, 96, 292, 42, 6, uiCardColor());
    tft.drawRoundRect(14, 96, 292, 42, 6, uiCardBorder());
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, uiCardColor());
    tft.drawString("Regen ABS", 24, 104);
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(145,165,175), uiCardColor());
    tft.drawString("releases regen if rear wheel locks", 24, 123);

    tft.setTextFont(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(tft.color565(120,145,155), tft.color565(7,12,16));
    tft.drawString("Swipe down for performance", 160, 191);
    tft.setTextDatum(TL_DATUM);
  }

  drawSettingsDynamic();
}

void updateActiveMenuValues() {
  if (uiScreen == UI_CONTROLLER) updateControllerValues();
  else if (uiScreen == UI_TRIP) updateTripValues();
  else if (uiScreen == UI_BATTERY) updateBatteryValues();
  else if (uiScreen == UI_SETTINGS) drawSettingsDynamic();
}

void drawActiveMenuPage() {
  if (uiScreen == UI_CONTROLLER) drawControllerPage();
  else if (uiScreen == UI_TRIP) drawTripPage();
  else if (uiScreen == UI_BATTERY) drawBatteryPage();
  else if (uiScreen == UI_SETTINGS) drawSettingsPage();

  uiNeedsFullRedraw = false;
  settingsControlsDirty = false;
}

void updateCurrentSlider(uint16_t x) {
  const int sx1 = 82;
  const int sx2 = 296;
  x = constrain((int)x, sx1, sx2);
  float ratio = (float)(x - sx1) / (float)(sx2 - sx1);
  userMaxDriveCurrentA =
      5.0f + ratio * (HARD_MAX_DRIVE_CURRENT_A - 5.0f);
  userMaxDriveCurrentA = roundf(userMaxDriveCurrentA);
  settingsDirty = true;
  settingsControlsDirty = true;
}

void updateFieldWeakeningSlider(uint16_t x) {
  const int sx1 = 82;
  const int sx2 = 296;
  x = constrain((int)x, sx1, sx2);
  float ratio = (float)(x - sx1) / (float)(sx2 - sx1);
  userFieldWeakeningA = ratio * HARD_MAX_FIELD_WEAKENING_A;
  userFieldWeakeningA =
      roundf(constrain(userFieldWeakeningA, 0.0f,
                       HARD_MAX_FIELD_WEAKENING_A) * 10.0f) / 10.0f;
  settingsDirty = true;
  settingsControlsDirty = true;
}

void updateRegenSlider(uint16_t x) {
  const int sx1 = 82;
  const int sx2 = 296;
  x = constrain((int)x, sx1, sx2);
  float ratio = (float)(x - sx1) / (float)(sx2 - sx1);
  userRegenBrakeA = ratio * HARD_MAX_REGEN_BRAKE_A;
  userRegenBrakeA =
      roundf(constrain(userRegenBrakeA, 0.0f,
                       HARD_MAX_REGEN_BRAKE_A) * 10.0f) / 10.0f;
  settingsDirty = true;
  settingsControlsDirty = true;
}

bool touchHitsSettingsControl(uint16_t x, uint16_t y) {
  if (settingsSection == 0) {
    return hit(x, y, 70, 80, 308, 134) ||
           hit(x, y, 70, 135, 308, 190) ||
           hit(x, y, 220, 37, 306, 78);
  }
  return hit(x, y, 70, 36, 308, 92) ||
         hit(x, y, 220, 95, 306, 140) ||
         hit(x, y, 35, 145, 285, 188);
}

void handleTouch(unsigned long now) {
  uint16_t x = 0, y = 0;
  bool down = readTouchScreen(x, y);

  if (saveBlockedMessage && now >= saveBlockedUntilMs) {
    saveBlockedMessage = false;
    settingsControlsDirty = true;
  }

  if (uiTransition != TRANS_NONE || saveRestartInProgress) {
    touchWasDown = down;
    return;
  }

  if (down) {
    settingsTouchLastX = x;
    settingsTouchLastY = y;
  }

  // Sliders follow the finger continuously.
  if (down && uiScreen == UI_SETTINGS && settingsSection == 0 &&
      hit(x, y, 70, 80, 308, 134)) {
    activeSettingsSlider = 1;
    updateCurrentSlider(x);
  }
  if (down && uiScreen == UI_SETTINGS && settingsSection == 0 &&
      hit(x, y, 70, 135, 308, 190)) {
    activeSettingsSlider = 2;
    updateFieldWeakeningSlider(x);
  }
  if (down && uiScreen == UI_SETTINGS && settingsSection == 1 &&
      hit(x, y, 70, 36, 308, 92)) {
    activeSettingsSlider = 3;
    updateRegenSlider(x);
  }

  if (down && !touchWasDown && (now - lastTouchMs) > 120) {
    lastTouchMs = now;
    settingsTouchStartX = x;
    settingsTouchStartY = y;
    settingsTouchLastX = x;
    settingsTouchLastY = y;
    settingsSwipeCandidate =
        uiScreen == UI_SETTINGS && !touchHitsSettingsControl(x, y);

    if (uiScreen == UI_MAIN) {
      if (hit(x, y,
              LOGO_TOUCH_X1, LOGO_TOUCH_Y1,
              LOGO_TOUCH_X2, LOGO_TOUCH_Y2)) {
        setUiScreen(UI_CONTROLLER);
      }
    } else {
      if (hit(x, y, 278, 0, 319, 34)) {
        setUiScreen(UI_MAIN);
      }
      else if (hit(x, y, 0, 194, 52, 239)) {
        previousMenuPage();
      }
      else if (hit(x, y, 267, 194, 319, 239)) {
        nextMenuPage();
      }
      else if (uiScreen == UI_SETTINGS && x >= 302 && y >= 35 && y <= 194) {
        uint8_t target = y < 115 ? 0 : 1;
        if (target != settingsSection) {
          startSettingsScroll(target,
              target > settingsSection ? TRANS_SETTINGS_UP : TRANS_SETTINGS_DOWN);
        }
      }
      else if (uiScreen == UI_SETTINGS && settingsSection == 0 &&
               hit(x, y, 220, 37, 306, 78)) {
        policeMode = !policeMode;
        settingsDirty = true;
        settingsControlsDirty = true;
      }
      else if (uiScreen == UI_SETTINGS && settingsSection == 1 &&
               hit(x, y, 220, 95, 306, 140)) {
        regenAbsEnabled = !regenAbsEnabled;
        settingsDirty = true;
        settingsControlsDirty = true;
      }
      else if (uiScreen == UI_SETTINGS && settingsSection == 1 &&
               hit(x, y, 35, 145, 285, 188)) {
        requestSaveAndRestart(now);
      }
    }
  }

  // Slider release: apply immediately to VESC RAM, but do not persist until
  // SAVE & RESTART is pressed.
  if (!down && touchWasDown && activeSettingsSlider != 0) {
    maxCurrentApplyPending = true;
    fieldWeakeningApplyPending = true;
    if (vescSeen) applyControllerSettingsToVesc(false);
    activeSettingsSlider = 0;
    settingsControlsDirty = true;
  }

  // Vertical swipe between the two settings sections.
  if (!down && touchWasDown && settingsSwipeCandidate &&
      uiScreen == UI_SETTINGS) {
    int dy = settingsTouchLastY - settingsTouchStartY;
    if (dy < -32 && settingsSection == 0) {
      startSettingsScroll(1, TRANS_SETTINGS_UP);
    } else if (dy > 32 && settingsSection == 1) {
      startSettingsScroll(0, TRANS_SETTINGS_DOWN);
    }
  }

  if (!down) settingsSwipeCandidate = false;
  touchWasDown = down;
}

void checkvalues() {
  total_km = startup_total_km + trip;
  bool traveled_enough_distance = (total_km - last_total_km_stored >= EEPROM_UPDATE_EACH_KM);
  if (traveled_enough_distance) {
    last_total_km_stored = total_km;
    EEPROM_writeAnything(EEPROM_MAGIC_VALUE, total_km);
  }
}

int readThrottleFiltered() {
  int raw = analogRead(THROTTLE_PIN);
  throttleMv = analogReadMilliVolts(THROTTLE_PIN);

  if (!throttleFilterInitialized) {
    throttleFiltered = raw;
    throttleFilterInitialized = true;
  } else {
    // Light low-pass filter. Keeps response fast but removes ADC jitter.
    throttleFiltered = (throttleFiltered * 0.75f) + ((float)raw * 0.25f);
  }

  throttleRaw = (int)(throttleFiltered + 0.5f);

  #ifdef DEBUG_MODE
  static unsigned long lastThrottleDebugMs = 0;
  if (millis() - lastThrottleDebugMs >= 100) {
    lastThrottleDebugMs = millis();
    Serial.print("THR raw=");
    Serial.print(throttleRaw);
    Serial.print(" mV=");
    Serial.print(throttleMv);
    Serial.print(" idleRaw=");
    Serial.print(throttleIdleRaw);
    Serial.print(" idleMv=");
    Serial.print(throttleIdleMv);
    Serial.print(" pct=");
    Serial.print((int)(throttlePercent * 100.0f));
    Serial.print(" cmdA=");
    Serial.print(commandedCurrentA, 1);
    Serial.print(" fault=");
    Serial.println(throttleFaultReason);
  }
  #endif

  return throttleRaw;
}

void resetIdleCalibration() {
  throttleIdleCalibrated = false;
  throttleIdleMv = 0;
  throttleCalSum = 0;
  throttleCalSamples = 0;
  throttleCalMin = 4095;
  throttleCalMax = 0;
  throttleCalStartMs = millis();
  throttleArmed = false;
  throttleDirection = 0;
  throttlePercent = 0.0f;
  throttleReleasedSinceMs = 0;
  throttleBadSinceMs = 0;
  throttleFaultReason = "NONE";
  commandedCurrentA = 0.0f;
}

void updateIdleCalibration(int raw, unsigned long now) {
  if (throttleIdleCalibrated) return;

  // Refuse to learn an input that is stuck at either ADC rail.
  if (raw <= THROTTLE_IDLE_MIN_RAW || raw >= THROTTLE_IDLE_MAX_RAW) {
    throttleCalSamples = 0;
    throttleCalSum = 0;
    throttleCalMin = 4095;
    throttleCalMax = 0;
    throttleCalStartMs = now;
    return;
  }

  if (throttleCalSamples == 0) {
    throttleCalStartMs = now;
    throttleCalMin = raw;
    throttleCalMax = raw;
  }

  if (raw < throttleCalMin) throttleCalMin = raw;
  if (raw > throttleCalMax) throttleCalMax = raw;
  throttleCalSum += raw;
  throttleCalSamples++;

  // If the throttle moves while calibrating, restart the idle learning window.
  if ((throttleCalMax - throttleCalMin) > THROTTLE_IDLE_STABILITY_RAW) {
    throttleCalSamples = 0;
    throttleCalSum = 0;
    throttleCalMin = 4095;
    throttleCalMax = 0;
    throttleCalStartMs = now;
    return;
  }

  if ((now - throttleCalStartMs) >= THROTTLE_IDLE_CAL_TIME_MS && throttleCalSamples >= 10) {
    throttleIdleRaw = (int)(throttleCalSum / throttleCalSamples);
    throttleIdleMv = throttleMv;
    throttleIdleCalibrated = true;
    throttleReleasedSinceMs = now;

    #ifdef DEBUG_MODE
    Serial.print("Throttle idle learned: RAW=");
    Serial.print(throttleIdleRaw);
    Serial.print(" mV=");
    Serial.println(throttleIdleMv);
    #endif
  }
}

void disarmThrottle() {
  throttleArmed = false;
  throttleDirection = 0;
  throttlePercent = 0.0f;
  throttleReleasedSinceMs = 0;
  commandedCurrentA = 0.0f;
}

bool updateSignalPlausibility(int raw, unsigned long now) {
  // Use calibrated millivolts rather than RAW ADC counts.
  // Your real signal is about 1132 mV released and 3145 mV full.
  static int prevMv = -1;

  bool railBad = (throttleMv <= THROTTLE_BAD_LOW_MV ||
                  throttleMv >= THROTTLE_BAD_HIGH_MV);

  bool jumpBad = false;
  if (prevMv >= 0) {
    int stepMv = throttleMv - prevMv;
    if (stepMv < 0) stepMv = -stepMv;
    jumpBad = stepMv > THROTTLE_MAX_STEP_MV;
  }
  prevMv = throttleMv;

  if (railBad || jumpBad) {
    if (throttleBadSinceMs == 0) throttleBadSinceMs = now;
  } else {
    throttleBadSinceMs = 0;
    throttleFaultReason = "NONE";
  }

  if (throttleBadSinceMs != 0 &&
      (now - throttleBadSinceMs) >= THROTTLE_BAD_TIME_MS) {
    throttleFaultReason = railBad ? "VOLT RAIL" : "VOLT JUMP";
    return false;
  }

  return true;
}

float calculateThrottlePercent(int raw) {
  if (!throttleIdleCalibrated || !throttleArmed) return 0.0f;

  int deltaMv = throttleMv - throttleIdleMv;

  // Ninebot Hall throttle rises with throttle. Keep direction detection as a
  // sanity check in case wiring is ever changed.
  if (throttleDirection == 0) {
    if (deltaMv >= THROTTLE_DIRECTION_DETECT_MV) throttleDirection = +1;
    else if (deltaMv <= -THROTTLE_DIRECTION_DETECT_MV) throttleDirection = -1;
    else return 0.0f;
  }

  float activeMv = (float)(deltaMv * throttleDirection);

  if (activeMv <= THROTTLE_DEADZONE_MV) return 0.0f;

  float usableSpanMv = THROTTLE_FULL_MV - (float)throttleIdleMv - THROTTLE_DEADZONE_MV;
  if (usableSpanMv < 500.0f) return 0.0f;

  activeMv -= THROTTLE_DEADZONE_MV;
  float value = activeMv / usableSpanMv;

  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return value;
}

void updateThrottleControl(unsigned long now) {
  int raw = readThrottleFiltered();

  if (saveRestartInProgress) {
    eBrakeActive = false;
    commandedCurrentA = 0.0f;
    UART.setCurrent(0.0f);
    return;
  }

  throttleSignalFault = !updateSignalPlausibility(raw, now);
  if (throttleSignalFault) {
    disarmThrottle();
    eBrakeActive = false;
    UART.setCurrent(0.0f);
    return;
  }

  if (!throttleIdleCalibrated) {
    updateIdleCalibration(raw, now);
    eBrakeActive = false;
    UART.setCurrent(0.0f);
    return;
  }

  if (vescSeen && UART.data.error != 0) {
    disarmThrottle();
    eBrakeActive = false;
    UART.setCurrent(0.0f);
    return;
  }

  if (!throttleArmed) {
    int diffMv = throttleMv - throttleIdleMv;
    if (diffMv < 0) diffMv = -diffMv;

    if (diffMv <= THROTTLE_RELEASE_MARGIN_MV) {
      if (throttleReleasedSinceMs == 0) throttleReleasedSinceMs = now;
      if ((now - throttleReleasedSinceMs) >= THROTTLE_ARM_TIME_MS) {
        throttleArmed = true;
      }
    } else {
      throttleReleasedSinceMs = 0;
    }

    eBrakeActive = false;
    UART.setCurrent(0.0f);
    return;
  }

  throttlePercent = calculateThrottlePercent(raw);
  bool freshTelemetry = vescSeen && ((now - lastVescRxMs) < 300);
  float absSpeed = fabsf(speed);

  // Adjustable release regen.
  if (releaseEBrakeEnabled && throttlePercent <= 0.001f && freshTelemetry &&
      absSpeed > E_BRAKE_MIN_SPEED_KMH && userRegenBrakeA > 0.05f) {

    eBrakeActive = true;

    float speedFactor = 1.0f;
    if (absSpeed < E_BRAKE_FULL_STRENGTH_KMH) {
      speedFactor =
          (absSpeed - E_BRAKE_MIN_SPEED_KMH) /
          (E_BRAKE_FULL_STRENGTH_KMH - E_BRAKE_MIN_SPEED_KMH);
      speedFactor = constrain(speedFactor, 0.0f, 1.0f);
    }

    float voltageFactor = 1.0f;
    if (UART.data.inpVoltage >= E_BRAKE_VOLTAGE_CUTOFF) {
      voltageFactor = 0.0f;
    } else if (UART.data.inpVoltage > E_BRAKE_VOLTAGE_TAPER_START) {
      voltageFactor =
          (E_BRAKE_VOLTAGE_CUTOFF - UART.data.inpVoltage) /
          (E_BRAKE_VOLTAGE_CUTOFF - E_BRAKE_VOLTAGE_TAPER_START);
      voltageFactor = constrain(voltageFactor, 0.0f, 1.0f);
    }

    float absFactor = 1.0f;
    if (regenAbsEnabled && regenAbsReleaseUntilMs != 0) {
      if (now < regenAbsReleaseUntilMs) {
        absFactor = 0.0f;
      } else {
        unsigned long sinceRelease = now - regenAbsReleaseUntilMs;
        if (sinceRelease < REGEN_ABS_REAPPLY_MS) {
          absFactor = (float)sinceRelease / (float)REGEN_ABS_REAPPLY_MS;
        } else {
          regenAbsReleaseUntilMs = 0;
        }
      }
    }

    float brakeA =
        userRegenBrakeA * speedFactor * voltageFactor * absFactor;
    commandedCurrentA = 0.0f;

    if (brakeA > 0.10f) UART.setBrakeCurrent(brakeA);
    else UART.setCurrent(0.0f);
    return;
  }

  eBrakeActive = false;
  if (throttlePercent > 0.001f) regenAbsReleaseUntilMs = 0;

  float targetCurrentA = throttlePercent * userMaxDriveCurrentA;

  if (policeMode) {
    float policeLimit = constrain(userPoliceLimitKmh,
                                  MIN_POLICE_LIMIT_KMH,
                                  MAX_POLICE_LIMIT_KMH);
    float taperStart = max(5.0f, policeLimit - 2.0f);
    if (speed >= policeLimit) {
      targetCurrentA = 0.0f;
    } else if (speed > taperStart) {
      float factor = (policeLimit - speed) / (policeLimit - taperStart);
      targetCurrentA *= constrain(factor, 0.0f, 1.0f);
    }
  }

  float dt = (lastControlMs == 0)
                 ? 0.02f
                 : ((float)(now - lastControlMs) / 1000.0f);
  dt = constrain(dt, 0.001f, 0.10f);

  if (targetCurrentA > commandedCurrentA) {
    float maxIncrease = userCurrentRiseAPerSec * dt;
    commandedCurrentA += maxIncrease;
    if (commandedCurrentA > targetCurrentA) commandedCurrentA = targetCurrentA;
  } else {
    commandedCurrentA = targetCurrentA;
  }

  commandedCurrentA =
      constrain(commandedCurrentA, 0.0f, userMaxDriveCurrentA);
  UART.setCurrent(commandedCurrentA);
}


void updateTelemetry(unsigned long now) {
  bool ok = UART.getVescValues();
  if (!ok) return;

  vescSeen = true;
  lastVescRxMs = now;

  tacho = (UART.data.tachometerAbs / (MOTOR_POLES * 3));
  rpm = (UART.data.rpm / (MOTOR_POLES / 2));
  trip = tacho / 1000.0f;
  wheel_diameter = (PI * WHEEL_DIAMETER_MM / 1000.0f);

  float measuredSpeed =
      ((rpm * wheel_diameter * GEAR_RAITO) / 1000.0f) * 60.0f;
  if (isfinite(measuredSpeed) &&
      fabsf(measuredSpeed) <= MAX_VALID_SPEED_KMH) {
    speed = measuredSpeed;
  }

  watts = UART.data.inpVoltage * UART.data.avgInputCurrent;

  if (displayBatteryVoltage <= 1.0f || !isfinite(displayBatteryVoltage)) {
    displayBatteryVoltage = UART.data.inpVoltage;
  } else {
    // About 0.7 s time constant at the normal telemetry rate. This prevents the
    // battery bar/range from jumping up and down with every throttle pulse.
    displayBatteryVoltage +=
        (UART.data.inpVoltage - displayBatteryVoltage) * 0.08f;
  }

  float absSpeed = fabsf(speed);

  // Session distance is integrated from speed. The old code divided the whole
  // VESC tachometer distance by only this session's moving time, causing values
  // such as 450 km/h after a short throttle press.
  if (lastDistanceIntegrationMs != 0) {
    unsigned long dtMs = now - lastDistanceIntegrationMs;
    if (dtMs > 0 && dtMs < 1000 && absSpeed > 0.3f) {
      sessionTripKm += absSpeed * ((float)dtMs / 3600000.0f);
    }
  }
  lastDistanceIntegrationMs = now;

  if (absSpeed > sessionMaxSpeed) sessionMaxSpeed = absSpeed;

  bool shouldBeMoving = absSpeed > 1.0f;
  if (shouldBeMoving && !movingNow) {
    movingNow = true;
    movingStartMs = now;
  } else if (!shouldBeMoving && movingNow) {
    movingAccumMs += (now - movingStartMs);
    movingNow = false;
  }

  // Regen ABS: detect a sudden rear-wheel speed collapse while braking.
  if (previousAbsSpeedMs != 0) {
    float dt = (float)(now - previousAbsSpeedMs) / 1000.0f;
    if (dt >= 0.02f && dt <= 0.30f) {
      float speedDrop = previousAbsSpeedKmh - absSpeed;
      float rawDecel = speedDrop > 0.0f ? speedDrop / dt : 0.0f;
      filteredWheelDecelKmhps =
          filteredWheelDecelKmhps * 0.65f + rawDecel * 0.35f;

      bool rapidDrop = speedDrop >= REGEN_ABS_SPEED_DROP_KMH;
      bool highDecel =
          filteredWheelDecelKmhps >= REGEN_ABS_DECEL_TRIGGER_KMHPS;

      if (regenAbsEnabled && eBrakeActive && absSpeed > 4.0f &&
          (rapidDrop || highDecel)) {
        regenAbsReleaseUntilMs = now + REGEN_ABS_RELEASE_MS;
      }
    }
  }
  previousAbsSpeedKmh = absSpeed;
  previousAbsSpeedMs = now;

  checkvalues();

  // Re-apply saved VESC values in RAM after boot/reconnect. Persistence is
  // performed only by the explicit SAVE & RESTART button.
  if (now > 1500 &&
      (maxCurrentApplyPending || fieldWeakeningApplyPending ||
       vescStoreApplyPending)) {
    applyControllerSettingsToVesc(vescStoreApplyPending);
  }
}


void sendAndroidAppTelemetry(unsigned long now) {
  if (!navBleConnected || navBleTx == nullptr) return;
  if ((now - lastAppTelemetryMs) < APP_TELEMETRY_INTERVAL_MS) return;
  lastAppTelemetryMs = now;

  char line[176];
  int n = snprintf(
      line, sizeof(line),
      "TEL|S=%.1f|V=%.1f|BIN=%.1f|MOTOR=%.1f|TM=%.0f|TV=%.0f|FAULT=%d|ARMED=%d|EB=%d|POLICE=%d",
      fabsf(speed),
      UART.data.inpVoltage,
      UART.data.avgInputCurrent,
      UART.data.avgMotorCurrent,
      UART.data.tempMotor,
      UART.data.tempMosfet,
      (vescSeen && ((now - lastVescRxMs) <= 750UL)) ? (int)UART.data.error : -1,
      throttleArmed ? 1 : 0,
      eBrakeActive ? 1 : 0,
      policeMode ? 1 : 0);

  if (n <= 0) return;
  if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
  navBleTx->setValue((uint8_t *)line, (size_t)n);
  navBleTx->notify();
}


void updateBacklight() {
  // Fixed full brightness. Configure once; repeatedly calling analogWrite every
  // frame can re-touch PWM hardware and waste time on the display path.
  static bool configured = false;
  if (configured) return;

  brightness = 255;
  pinMode(LCD_BACK_LIGHT_PIN, OUTPUT);
  analogWrite(LCD_BACK_LIGHT_PIN, brightness);
  configured = true;
}




// -----------------------------------------------------------------------------
// CLASSIC SIMPLE VESC DISPLAY UI
// -----------------------------------------------------------------------------
// This restores the main dashboard layout from the earlier project versions:
//   - huge centre speed
//   - battery voltage top-left
//   - watts / VESC error top-right
//   - VESC temp, motor temp, odometer, battery current and phase current below
//
// Android BLE control, touch menus, police mode, FW, e-brake, persistent
// settings, OSM navigation and the V22.1 timing/queue fixes remain unchanged.
// -----------------------------------------------------------------------------

bool classicNavigationFresh() {
  return navBleConnected &&
         navState.active &&
         ((millis() - navState.lastUpdateMs) <= NAV_STALE_MS);
}

uint16_t classicVescTempColor(float tempC) {
  if (tempC > VESC_TEMP_WARNING2) return TFT_RED;
  if (tempC > VESC_TEMP_WARNING1) return TFT_YELLOW;
  return TFT_GREEN;
}

uint16_t classicMotorTempColor(float tempC) {
  if (tempC > MOTOR_TEMP_WARNING2) return TFT_RED;
  if (tempC > MOTOR_TEMP_WARNING1) return TFT_YELLOW;
  return TFT_GREEN;
}

uint16_t classicBatteryColor(float volts) {
  if (volts > BATTERY_WARNING_HIGH) return TFT_RED;
  if (volts < BATTERY_WARNING_0) return TFT_RED;
  if (volts < BATTERY_WARNING_LOW) return TFT_YELLOW;
  return TFT_GREEN;
}

void classicDrawBackgroundImage() {
  tft.fillScreen(TFT_BLACK);

#ifdef DO_LOGO_DRAW
  int16_t rc =
      png.openFLASH((uint8_t *)background_image, sizeof(background_image), pngDraw);
  if (rc == PNG_SUCCESS) {
    tft.startWrite();
    png.decode(NULL, 0);
    tft.endWrite();
  }
  png.close();
#endif
}

void classicDrawStaticLabels(bool navigationMode) {
  // Restore the static labels used by the original SimpleVescDisplay screen.
  // Navigation only occupies the central/right mid-screen; top/bottom telemetry
  // stays in its original locations.
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(1);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 30);
  tft.print("Battery");

  // Watts/Error label is dynamic because it changes with VESC fault state.

  tft.setFreeFont(DATAFONTSMALLTEXT);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setCursor(15, 235);
  tft.print("VescT");

  tft.setCursor(65, 235);
  tft.print("MotorT");

  tft.setCursor(135, 235);
  tft.print("ODOMETER");

  tft.setCursor(215, 235);
  tft.print("BATT A");

  tft.setCursor(265, 235);
  tft.print("PHASE A");

  if (navigationMode) {
    tft.setTextFont(1);
    tft.setTextColor(tft.color565(160, 175, 182), TFT_BLACK);
    tft.setCursor(45, 180);
    tft.print("km/h");
  }

  tft.setTextDatum(TL_DATUM);
}

void classicDrawNavigationPanel(bool force) {
  static int oldDistance = -999;
  static int oldRemaining = -999;
  static int oldBearing = -999;
  static int oldEta = -999;
  static char oldTurn[9] = "";
  static char oldRoad[32] = "";
  static char oldArrival[6] = "";

  unsigned long now = millis();

  bool mapDue =
      force ||
      (navMapDirty &&
       (((now - navMapLastGeometryRxMs) >= NAV_MAP_SETTLE_MS) ||
        ((now - navMapLastDrawMs) >= NAV_MAP_MAX_REDRAW_MS)));

  bool guideDirty =
      force ||
      navState.distanceM != oldDistance ||
      navState.remainingM != oldRemaining ||
      navState.bearingDeg != oldBearing ||
      navState.etaMin != oldEta ||
      strncmp(navState.turn, oldTurn, sizeof(oldTurn) - 1) != 0 ||
      strncmp(navState.road, oldRoad, sizeof(oldRoad) - 1) != 0 ||
      strncmp(navState.arrival, oldArrival, 5) != 0;

  if (!mapDue && !guideDirty) return;

  const uint16_t panelBg = tft.color565(7, 12, 16);
  const uint16_t panelBorder = tft.color565(60, 78, 86);
  const uint16_t secondary = tft.color565(145, 160, 168);

  tft.startWrite();

  if (force) {
    tft.fillRoundRect(NAV_PANEL_X, NAV_PANEL_Y,
                      NAV_PANEL_W, NAV_PANEL_H, 6, panelBg);
    tft.drawRoundRect(NAV_PANEL_X, NAV_PANEL_Y,
                      NAV_PANEL_W, NAV_PANEL_H, 6, panelBorder);
  }

  if (mapDue) {
    // Close-up OSM road map, kept from the later navigation work but without
    // any Rage-style dashboard framing.
    drawNavigationRouteMap(NAV_PANEL_X + 4,
                           NAV_PANEL_Y + 33,
                           NAV_PANEL_W - 8,
                           77);
    navMapDirty = false;
    navMapLastDrawMs = now;
  }

  if (guideDirty) {
    // Header/turn area.
    tft.fillRect(NAV_PANEL_X + 3, NAV_PANEL_Y + 3,
                 NAV_PANEL_W - 6, 29, panelBg);

    drawNavigationTurnIcon(NAV_PANEL_X + 18,
                           NAV_PANEL_Y + 17,
                           navState.turn,
                           TFT_WHITE);

    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(DATAFONTSMALLTEXT);
    tft.setTextColor(TFT_WHITE, panelBg);

    String dist = compactNavDistance(navState.distanceM);
    tft.drawString(dist, NAV_PANEL_X + 36, NAV_PANEL_Y + 7);

    String road = String(navState.road);
    if (road.length() > 16) road = road.substring(0, 15) + ".";
    tft.setTextFont(1);
    tft.setTextColor(secondary, panelBg);
    tft.drawString(road, NAV_PANEL_X + 36, NAV_PANEL_Y + 20);

    // Footer.
    tft.fillRect(NAV_PANEL_X + 3, NAV_PANEL_Y + 112,
                 NAV_PANEL_W - 6, 28, panelBg);

    tft.setTextFont(1);
    tft.setTextColor(secondary, panelBg);

    String leftText = compactNavDistance(navState.remainingM);
    tft.drawString("LEFT " + leftText,
                   NAV_PANEL_X + 6, NAV_PANEL_Y + 116);

    if (navState.arrival[0] != '\0' &&
        strcmp(navState.arrival, "--:--") != 0) {
      tft.drawString(String("ETA ") + navState.arrival,
                     NAV_PANEL_X + 6, NAV_PANEL_Y + 129);
    } else {
      tft.drawString(String("ETA ") + navState.etaMin + " min",
                     NAV_PANEL_X + 6, NAV_PANEL_Y + 129);
    }

    oldDistance = navState.distanceM;
    oldRemaining = navState.remainingM;
    oldBearing = navState.bearingDeg;
    oldEta = navState.etaMin;

    strncpy(oldTurn, navState.turn, sizeof(oldTurn));
    oldTurn[sizeof(oldTurn) - 1] = '\0';

    strncpy(oldRoad, navState.road, sizeof(oldRoad));
    oldRoad[sizeof(oldRoad) - 1] = '\0';

    strncpy(oldArrival, navState.arrival, sizeof(oldArrival));
    oldArrival[sizeof(oldArrival) - 1] = '\0';
  }

  tft.endWrite();
}

void classicDrawDynamic(bool force, bool navigationMode) {
  static int oldSpeed = -999;
  static int oldVescTemp = -999;
  static int oldMotorTemp = -999;
  static int oldVoltageTenths = -999;
  static int oldWatts = -999999;
  static int oldFault = -999;
  static int oldPhaseTenths = -999999;
  static int oldBatteryTenths = -999999;
  static int oldOdometer = -999999;

  int speedINT = (int)roundf(displaySpeed);
  if (speedINT < 0) speedINT = 0;

  int vescTempINT = (int)roundf(UART.data.tempMosfet);
  int motorTempINT = (int)roundf(UART.data.tempMotor);
  int voltageTenths = (int)roundf(UART.data.inpVoltage * 10.0f);
  int wattsINT = (int)roundf(watts);
  int faultINT =
      (vescSeen && ((millis() - lastVescRxMs) <= 750UL))
          ? (int)UART.data.error
          : -1;
  int phaseTenths = (int)roundf(UART.data.avgMotorCurrent * 10.0f);
  int batteryTenths = (int)roundf(UART.data.avgInputCurrent * 10.0f);
  int odometerINT = (int)roundf(total_km);

  bool speedDirty = force || speedINT != oldSpeed;
  bool vescTempDirty = force || vescTempINT != oldVescTemp;
  bool motorTempDirty = force || motorTempINT != oldMotorTemp;
  bool voltageDirty = force || voltageTenths != oldVoltageTenths;
  bool powerDirty = force || wattsINT != oldWatts || faultINT != oldFault;
  bool phaseDirty = force || phaseTenths != oldPhaseTenths;
  bool batteryCurrentDirty = force || batteryTenths != oldBatteryTenths;
  bool odometerDirty = force || odometerINT != oldOdometer;

  // MAIN SPEED ----------------------------------------------------------------
  if (speedDirty) {
    uint16_t speedColor =
        (speedINT > HIGH_SPEED_WARNING) ? TFT_RED : TFT_WHITE;

    tft.setFreeFont(SPEEDFONT);

    if (navigationMode) {
      tft.setCursor(4, 162);
    } else {
      tft.setCursor(83, 162);
    }

    // This is the same technique used by the original project.
    // Do NOT fill a large rectangle behind the Jersey font.
    ClassicSpeed.setTextColor(speedColor, TFT_BLACK);
    ClassicSpeed.print(speedINT);

    if (navigationMode) {
      tft.setTextFont(1);
      tft.setTextColor(tft.color565(160, 175, 182), TFT_BLACK);
      tft.setCursor(45, 180);
      tft.print("km/h");
    }

    oldSpeed = speedINT;
  }

  // VESC TEMP -----------------------------------------------------------------
  if (vescTempDirty) {
    tft.setCursor(5, 220);
    tft.setFreeFont(DATAFONTSMALL);
    ClassicVescTemp.setTextColor(
        classicVescTempColor(UART.data.tempMosfet), TFT_BLACK);
    ClassicVescTemp.print(vescTempINT);
    oldVescTemp = vescTempINT;
  }

  // MOTOR TEMP ----------------------------------------------------------------
  if (motorTempDirty) {
    tft.setCursor(65, 220);
    tft.setFreeFont(DATAFONTSMALL);
    ClassicMotorTemp.setTextColor(
        classicMotorTempColor(UART.data.tempMotor), TFT_BLACK);
    ClassicMotorTemp.print(motorTempINT);
    oldMotorTemp = motorTempINT;
  }

  // BATTERY VOLTAGE ------------------------------------------------------------
  if (voltageDirty) {
    tft.setCursor(10, 25);
    tft.setFreeFont(DATAFONTSMALL2);
    ClassicBatteryV.setTextColor(
        classicBatteryColor(UART.data.inpVoltage), TFT_BLACK);
    ClassicBatteryV.print(UART.data.inpVoltage, 1);
    oldVoltageTenths = voltageTenths;
  }

  // WATTS / ERROR --------------------------------------------------------------
  if (powerDirty) {
    if (faultINT == 0) {
      tft.setCursor(260, 25);
      tft.setFreeFont(DATAFONTSMALL2);
      ClassicPower.setTextColor(TFT_WHITE, TFT_BLACK);
      ClassicPower.print(wattsINT);

      tft.setCursor(270, 30);
      tft.setTextFont(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.print("Watts");
    } else {
      tft.setCursor(270, 25);
      tft.setFreeFont(DATAFONTSMALL2);
      ClassicPower.setTextColor(TFT_RED, TFT_BLACK);
      ClassicPower.print(faultINT);

      tft.setCursor(270, 30);
      tft.setTextFont(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.print("Error");
    }

    oldWatts = wattsINT;
    oldFault = faultINT;
  }

  // PHASE CURRENT --------------------------------------------------------------
  if (phaseDirty) {
    tft.setCursor(270, 220);
    tft.setFreeFont(DATAFONTSMALL);
    ClassicPhaseA.setTextColor(TFT_GREEN, TFT_BLACK);
    ClassicPhaseA.print(UART.data.avgMotorCurrent, 0);
    oldPhaseTenths = phaseTenths;
  }

  // BATTERY CURRENT ------------------------------------------------------------
  if (batteryCurrentDirty) {
    tft.setCursor(220, 220);
    tft.setFreeFont(DATAFONTSMALL);
    ClassicBattA.setTextColor(TFT_GREEN, TFT_BLACK);
    ClassicBattA.print(UART.data.avgInputCurrent, 0);
    oldBatteryTenths = batteryTenths;
  }

  // ODOMETER -------------------------------------------------------------------
  if (odometerDirty) {
    tft.setCursor(145, 220);
    tft.setFreeFont(DATAFONTSMALL);
    ClassicOdometer.setTextColor(TFT_WHITE, TFT_BLACK);
    ClassicOdometer.print(odometerINT);
    oldOdometer = odometerINT;
  }

  tft.setTextDatum(TL_DATUM);

  if (navigationMode) {
    classicDrawNavigationPanel(force);
  }
}

void classicDrawMainDashboard() {
  static bool previousNavigationMode = false;

  bool navigationMode = classicNavigationFresh();

  if (navigationMode != previousNavigationMode) {
    // Full background redraw is only done when entering/leaving navigation,
    // not every display update.
    classicDashboardNeedsRedraw = true;
    previousNavigationMode = navigationMode;
  }

  bool force = classicDashboardNeedsRedraw;

  if (force) {
    classicDrawBackgroundImage();
    classicDrawStaticLabels(navigationMode);

    if (navigationMode) {
      // Establish a clean neutral navigation panel before dynamic content.
      const uint16_t panelBg = tft.color565(7, 12, 16);
      tft.fillRoundRect(NAV_PANEL_X, NAV_PANEL_Y,
                        NAV_PANEL_W, NAV_PANEL_H, 6, panelBg);
      tft.drawRoundRect(NAV_PANEL_X, NAV_PANEL_Y,
                        NAV_PANEL_W, NAV_PANEL_H, 6,
                        tft.color565(60, 78, 86));
    }

    classicDashboardNeedsRedraw = false;
  }

  classicDrawDynamic(force, navigationMode);
}

void drawDisplay() {
  updateBacklight();

  // Keep V22.1's faster visual smoothing. This changes display animation only;
  // motor control and Police mode always use the real VESC speed value.
  static unsigned long lastSpeedAnimMs = 0;
  unsigned long speedAnimNow = millis();

  float speedDt =
      (lastSpeedAnimMs == 0)
          ? 0.025f
          : (float)(speedAnimNow - lastSpeedAnimMs) / 1000.0f;
  lastSpeedAnimMs = speedAnimNow;
  speedDt = constrain(speedDt, 0.005f, 0.080f);

  float speedDelta = speed - displaySpeed;
  float response = (speedDelta >= 0.0f) ? 18.0f : 14.0f;
  float speedAlpha = 1.0f - expf(-response * speedDt);

  displaySpeed += speedDelta * speedAlpha;
  if (displaySpeed < 0.05f) displaySpeed = 0.0f;

  classicDrawMainDashboard();
}


void debugPrint(unsigned long now) {
  #ifdef DEBUG_MODE
  if ((now - lastDebugMs) < DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;

  Serial.print("THR raw=");
  Serial.print(throttleRaw);
  Serial.print(" mV=");
  Serial.print(throttleMv);
  Serial.print(" idle=");
  Serial.print(throttleIdleRaw);
  Serial.print(" cal=");
  Serial.print(throttleIdleCalibrated ? 1 : 0);
  Serial.print(" armed=");
  Serial.print(throttleArmed ? 1 : 0);
  Serial.print(" dir=");
  Serial.print(throttleDirection);
  Serial.print(" pct=");
  Serial.print(throttlePercent * 100.0f, 1);
  Serial.print(" fault=");
  Serial.print(throttleSignalFault ? 1 : 0);
  Serial.print(" VESC_RX=");
  Serial.print(vescSeen ? 1 : 0);
  Serial.print(" VESCerr=");
  Serial.print(vescSeen ? (int)UART.data.error : -1);
  Serial.print(" cmdA=");
  Serial.println(commandedCurrentA, 2);
  #endif
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup(void) {
  #ifdef DEBUG_MODE
  Serial.begin(115200);
  #endif

  analogReadResolution(12);
  analogSetPinAttenuation(THROTTLE_PIN, ADC_11db);
  pinMode(THROTTLE_PIN, INPUT);

  #ifdef DEBUG_MODE
  Serial.println("G30 throttle V5: measured 1132mV released -> 3145mV full.");
  Serial.println("V26: classic UI + expanded Android VESC settings protocol.");
  Serial.println("V5 uses millivolts for throttle mapping and fault detection.");
  #endif

  VescSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  UART.setSerialPort(&VescSerial);
  memset(&UART.data, 0, sizeof(UART.data));

  // Do not enable the ComEVesc raw debug stream here; it can add lots of serial
  // output and timing jitter. Our own debug line above is enough.

  // Send a zero-current command as soon as UART is available.
  UART.setCurrent(0.0f);

  tft.begin();
  EEPROM.begin(100);
  tft.setRotation(1);

  // Separate XPT2046 touch SPI bus on the CYD.
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  touch.begin(touchSPI);
  touch.setRotation(0);

  // BLE navigation runs independently from the VESC UART.
  setupNavigationBle();

  loadDisplaySettings();
  sessionStartMs = millis();
  tft.fillScreen(TFT_BLACK);

  EEPROM_readAnything(EEPROM_MAGIC_VALUE, startup_total_km);
  if (isnan(startup_total_km) || startup_total_km < 0.0f || startup_total_km > 1000000.0f) {
    tft.setCursor(40, 160);
    tft.setTextFont(4);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("SETUP EPROM...");

    // Only initialize the float we actually use. The original code wrote an
    // int at every byte address, which overlaps writes and can run past the
    // allocated EEPROM area.
    startup_total_km = 0.0f;
    EEPROM_writeAnything(EEPROM_MAGIC_VALUE, startup_total_km);
    delay(1500);
    ESP.restart();
  }

  // Try to get one telemetry packet. Motor output stays at zero regardless.
  if (UART.getVescValues()) {
    vescSeen = true;
    lastVescRxMs = millis();
  }

  last_total_km_stored = startup_total_km;
  tacho = (UART.data.tachometerAbs / (MOTOR_POLES * 3));
  trip = tacho / 1000.0f;
  if (startup_total_km != 0) {
    startup_total_km = startup_total_km - trip;
  }

  resetIdleCalibration();

  #ifdef DO_LOGO_DRAW
  int16_t rc_bg = png.openFLASH((uint8_t *)startup_image, sizeof(startup_image), pngDraw);
  if (rc_bg == PNG_SUCCESS) {
    tft.startWrite();
    rc_bg = png.decode(NULL, 0);
    tft.endWrite();
  }
  png.close();

  // Short startup splash. Throttle still arms only after a valid released
  // throttle has been learned, but we avoid a second calibration cycle.
  delay(350);

  UART.setCurrent(0.0f);

  tft.fillScreen(TFT_BLACK);
  #endif

  classicDashboardNeedsRedraw = true;
  lastControlMs = millis();
  lastKeepaliveMs = millis();
  lastTelemetryMs = millis();
  lastDisplayMs = millis();
  lastDistanceIntegrationMs = 0;
  previousAbsSpeedMs = 0;
}

// -----------------------------------------------------------------------------
// Main loop - NO blocking delay here. Throttle control must keep running.
// -----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1) Highest priority: throttle command at 50 Hz. BLE/map parsing is kept
  // below motor control so a burst of navigation packets cannot delay throttle.

  if ((now - lastControlMs) >= CONTROL_INTERVAL_MS) {
    updateThrottleControl(now);
    lastControlMs = now;
  }

  // 2) Keep VESC UART session alive as an extra heartbeat.
  if ((now - lastKeepaliveMs) >= KEEPALIVE_INTERVAL_MS) {
    UART.sendKeepalive();
    lastKeepaliveMs = now;
  }

  // 3) VESC telemetry. It is diagnostic/display data, not a throttle interlock.
  if ((now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
    updateTelemetry(now);
    lastTelemetryMs = now;
  }

  // 4) Bluetooth/navigation work is lower priority than motor control/UART.
  serviceNavigationBleConnection();
  serviceNavigationPackets();

  // Live data for the Android control app (notifications are ignored by the
  // navigation webpage when it does not subscribe to TX).
  sendAndroidAppTelemetry(now);

  // 5) Touch and UI animations are non-blocking.
  handleTouch(now);
  updateUiTransition(now);

  // Manual Save & Restart. Keep output at zero while the VESC stores config.
  if (saveRestartInProgress) {
    UART.setCurrent(0.0f);
    if ((now - saveRestartStartedMs) >= SAVE_RESTART_DELAY_MS) {
      ESP.restart();
    }
  }

  // 6) Display.
  if ((now - lastDisplayMs) >= DISPLAY_INTERVAL_MS) {
    if (uiTransition != TRANS_NONE) {
      // Animation owns the display for ~135 ms. Motor control still runs above.
    }
    else if (uiScreen == UI_MAIN) {
      drawDisplay();
    } else {
      static unsigned long lastMenuValueRefreshMs = 0;

      // Full menu redraw ONLY when entering/changing page.
      if (uiNeedsFullRedraw) {
        drawActiveMenuPage();
        lastMenuValueRefreshMs = now;
      }
      // Settings slider/toggle can refresh quickly, but only their small areas.
      else if (uiScreen == UI_SETTINGS &&
               (settingsControlsDirty ||
                (now - lastMenuValueRefreshMs) >= 120)) {
        drawSettingsDynamic();
        lastMenuValueRefreshMs = now;
      }
      // Other live values refresh at ~8 Hz in small value rectangles.
      else if ((now - lastMenuValueRefreshMs) >= 120) {
        updateActiveMenuValues();
        lastMenuValueRefreshMs = now;
      }
    }

    lastDisplayMs = now;
  }

  debugPrint(now);
}
