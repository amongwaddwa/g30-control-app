package se.g30.control;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.Gravity;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Space;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Native Android control panel for the matching G30 ESP32 firmware.
 *
 * It exposes the normal G30 ride controls plus a large Advanced VESC section.
 * Advanced controls use documented LispBM conf-set parameters and are sent only
 * when the user actually moves a control. Motor detection, FOC R/L/flux/KP/KI,
 * sensor mode and app/CAN configuration remain intentionally excluded.
 */
public final class MainActivity extends Activity implements G30BleClient.Listener {

    private static final int REQUEST_BT_PERMISSIONS = 1001;
    private static final int REQUEST_ENABLE_BT = 1002;
    private static final String NAVIGATION_URL =
            "https://amongwaddwa.github.io/g30-nav/";

    private static final int BG = Color.rgb(7, 12, 16);
    private static final int CARD = Color.rgb(18, 29, 36);
    private static final int CARD_EDGE = Color.rgb(52, 72, 82);
    private static final int TEXT = Color.rgb(242, 247, 249);
    private static final int MUTED = Color.rgb(151, 171, 181);
    private static final int ACCENT = Color.rgb(0, 210, 184);
    private static final int ORANGE = Color.rgb(255, 157, 50);
    private static final int RED = Color.rgb(255, 74, 74);
    private static final int GREEN = Color.rgb(65, 214, 111);

    private G30BleClient ble;
    private boolean connected;
    private final Handler actionHandler = new Handler(Looper.getMainLooper());
    private final List<AdvancedBinding> advancedBindings = new ArrayList<>();
    private int advancedSequenceGeneration = 0;

    private TextView connectionStatus;
    private Button connectButton;
    private Button readButton;
    private Button applyButton;
    private Button saveButton;

    private TextView speedValue;
    private TextView voltageValue;
    private TextView batteryCurrentValue;
    private TextView motorCurrentValue;
    private TextView motorTempValue;
    private TextView vescTempValue;
    private TextView faultValue;
    private TextView driveStateValue;

    private SettingSlider driveCurrent;
    private SettingSlider fieldWeakening;
    private SettingSlider releaseRegen;
    private SettingSlider batteryCurrent;
    private SettingSlider batteryRegen;
    private SettingSlider policeSpeed;
    private SettingSlider accelerationRamp;
    private SettingSlider cutoffStart;
    private SettingSlider cutoffEnd;

    private Switch policeSwitch;
    private Switch releaseBrakeSwitch;
    private Switch regenAbsSwitch;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Window window = getWindow();
        window.setStatusBarColor(BG);
        window.setNavigationBarColor(BG);
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        ble = new G30BleClient(this, this);
        setContentView(createUi());
        setConnectedUi(false, "Inte ansluten");
    }

    private View createUi() {
        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        scroll.setBackgroundColor(BG);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(dp(14), dp(14), dp(14), dp(32));
        scroll.addView(root, new ScrollView.LayoutParams(
                ScrollView.LayoutParams.MATCH_PARENT,
                ScrollView.LayoutParams.WRAP_CONTENT
        ));

        LinearLayout header = card();
        TextView title = text("G30 CONTROL", 27, TEXT, true);
        title.setLetterSpacing(0.08f);
        header.addView(title);
        header.addView(text("ESP32 • UBOX • Bluetooth", 13, MUTED, false));

        connectionStatus = text("Inte ansluten", 14, ORANGE, true);
        connectionStatus.setPadding(0, dp(12), 0, dp(8));
        header.addView(connectionStatus);

        LinearLayout headerButtons = horizontal();
        connectButton = button("ANSLUT", ACCENT, Color.rgb(2, 31, 27));
        readButton = button("LÄS INSTÄLLNINGAR", CARD_EDGE, TEXT);
        headerButtons.addView(connectButton, weighted());
        headerButtons.addView(space(dp(8)));
        headerButtons.addView(readButton, weighted());
        header.addView(headerButtons);
        root.addView(header);

        connectButton.setOnClickListener(v -> {
            if (connected) {
                ble.disconnect();
                setConnectedUi(false, "Frånkopplad");
            } else {
                beginConnect();
            }
        });
        readButton.setOnClickListener(v -> ble.sendLine("CFG|GET"));

        addSectionTitle(root, "LIVE DATA");
        LinearLayout telemetry = card();
        speedValue = telemetryBig(telemetry, "SPEED", "-- km/h", ACCENT);

        LinearLayout row1 = horizontal();
        voltageValue = stat(row1, "VOLT", "-- V", ORANGE);
        batteryCurrentValue = stat(row1, "BATTERY", "-- A", TEXT);
        motorCurrentValue = stat(row1, "MOTOR", "-- A", TEXT);
        telemetry.addView(row1);

        LinearLayout row2 = horizontal();
        motorTempValue = stat(row2, "MOTOR TEMP", "-- °C", TEXT);
        vescTempValue = stat(row2, "VESC TEMP", "-- °C", TEXT);
        faultValue = stat(row2, "FAULT", "--", GREEN);
        telemetry.addView(row2);

        driveStateValue = text("Väntar på telemetri", 13, MUTED, false);
        driveStateValue.setPadding(0, dp(10), 0, 0);
        telemetry.addView(driveStateValue);
        root.addView(telemetry);

        addSectionTitle(root, "PERFORMANCE");
        driveCurrent = new SettingSlider(
                "Drive current",
                "Max positiv motorström. VESC Tool är fortfarande den hårda gränsen.",
                5f, 35f, 1f, 25f, ACCENT
        );
        root.addView(driveCurrent.view);

        fieldWeakening = new SettingSlider(
                "Field weakening",
                "Ger mer toppfart men kan öka motor- och controllervärme.",
                0f, 35f, 0.5f, 0f, ORANGE
        );
        root.addView(fieldWeakening.view);

        accelerationRamp = new SettingSlider(
                "Acceleration response",
                "Hur snabbt begärd motorström får stiga, i A/s.",
                20f, 120f, 5f, 60f, ACCENT
        );
        root.addView(accelerationRamp.view);

        policeSwitch = addToggle(
                root,
                "Police mode",
                "Begränsar drivningen mjukt vid vald hastighet.",
                false
        );

        policeSpeed = new SettingSlider(
                "Police speed",
                "Mjuk hastighetsgräns när Police mode är på.",
                10f, 25f, 1f, 20f, ACCENT
        );
        root.addView(policeSpeed.view);

        addSectionTitle(root, "BRAKE & BATTERY");
        releaseBrakeSwitch = addToggle(
                root,
                "E-brake on throttle release",
                "Aktiverar mild regenerativ broms när gasen släpps.",
                true
        );

        releaseRegen = new SettingSlider(
                "Release e-brake",
                "Motorbromsens styrka. Batteriets regen-gräns ställs separat.",
                0f, 12f, 0.5f, 2f, Color.CYAN
        );
        root.addView(releaseRegen.view);

        regenAbsSwitch = addToggle(
                root,
                "Regen ABS",
                "ESP32 modulerar release-regen vid snabb hjulhastighetsförlust.",
                true
        );

        batteryCurrent = new SettingSlider(
                "Battery current max",
                "Ström från originalbatteriet. Börja omkring 15 A.",
                5f, 25f, 1f, 15f, ORANGE
        );
        root.addView(batteryCurrent.view);

        batteryRegen = new SettingSlider(
                "Battery regen max",
                "Max laddström tillbaka till batteriet vid regen.",
                0f, 8f, 0.5f, 3f, Color.CYAN
        );
        root.addView(batteryRegen.view);

        cutoffStart = new SettingSlider(
                "Voltage cutoff start",
                "VESC börjar minska drivningen under denna spänning.",
                30f, 36f, 0.5f, 34f, ORANGE
        );
        root.addView(cutoffStart.view);

        cutoffEnd = new SettingSlider(
                "Voltage cutoff end",
                "Ingen drivström under denna spänning. Måste ligga minst 1 V under start.",
                28f, 33f, 0.5f, 30f, RED
        );
        root.addView(cutoffEnd.view);

        addSectionTitle(root, "ADVANCED VESC");
        LinearLayout advancedWarning = card();
        advancedWarning.addView(text("BARA ÄNDRADE REGLAGE SKICKAS", 15, ORANGE, true));
        advancedWarning.addView(text(
                "De här reglagen visar startvärden i appen, inte en automatisk avläsning av VESC. " +
                        "Ett avancerat värde skickas bara om du själv flyttar just det reglaget. " +
                        "Det minskar risken att appen skriver över en befintlig VESC-inställning av misstag.",
                12, MUTED, false
        ));
        root.addView(advancedWarning);

        addSectionTitle(root, "ADVANCED • CURRENT / POWER");
        addAdvanced(root, "CURS", "Motor current scale",
                "Skalar VESC motorström 20–100 % utan att ändra huvudgränsen.",
                20f, 100f, 5f, 100f, ACCENT, 0.01f);
        addAdvanced(root, "BRS", "Brake current scale",
                "Skalar VESC bromsström 20–100 %.",
                20f, 100f, 5f, 100f, Color.CYAN, 0.01f);
        addAdvanced(root, "WMAX", "Maximum wattage",
                "Extra effektgräns i VESC. Current limits gäller fortfarande.",
                250f, 1500f, 50f, 750f, ORANGE, 1f);
        addAdvanced(root, "WBRAKE", "Maximum brake wattage",
                "Positivt appvärde; ESP32 skickar negativ l-watt-min till VESC.",
                0f, 500f, 25f, 250f, Color.CYAN, 1f);

        addSectionTitle(root, "ADVANCED • SPEED / DUTY");
        addAdvanced(root, "DUTY", "Maximum duty cycle",
                "VESC hard duty ceiling. 95 % är övre gränsen i denna app.",
                60f, 95f, 1f, 95f, ORANGE, 0.01f);
        addAdvanced(root, "MINDUTY", "Minimum duty cycle",
                "Låg-nivå VESC-inställning. Ändra bara om du vet varför.",
                0f, 5f, 0.5f, 0.5f, RED, 0.01f);
        addAdvanced(root, "ERPM", "Maximum ERPM",
                "Elektrisk varvtalsgräns. Appen begränsar området till 10k–30k ERPM.",
                10000f, 30000f, 500f, 20000f, ORANGE, 1f);
        addAdvanced(root, "ERPMS", "ERPM limit start",
                "När VESC börjar mjuka av mot ERPM-gränsen.",
                50f, 95f, 5f, 80f, ACCENT, 0.01f);
        addAdvanced(root, "VMAX", "VESC hard speed limit",
                "km/h via VESC max-speed. Kräver korrekta SI motor/wheel-inställningar.",
                10f, 60f, 1f, 45f, RED, 1f);
        addAdvanced(root, "FWDUTY", "Field weakening duty start",
                "Duty där VESC börjar använda field weakening.",
                70f, 95f, 1f, 85f, ORANGE, 0.01f);

        addSectionTitle(root, "ADVANCED • VOLTAGE / TEMP");
        addAdvanced(root, "VINMIN", "Minimum input voltage",
                "VESC minimum input-voltage fault limit.",
                28f, 34f, 0.5f, 30f, ORANGE, 1f);
        addAdvanced(root, "VINMAX", "Maximum input voltage",
                "VESC maximum input-voltage fault limit.",
                40f, 45f, 0.5f, 44f, ORANGE, 1f);
        addAdvanced(root, "TMSTART", "Motor temp cutoff start",
                "Temperatur där VESC börjar minska motorström.",
                60f, 90f, 1f, 80f, ORANGE, 1f);
        addAdvanced(root, "TMEND", "Motor temp cutoff end",
                "Temperatur där VESC stoppar motorström helt.",
                75f, 105f, 1f, 100f, RED, 1f);
        addAdvanced(root, "TACC", "Acceleration temp decrease",
                "Sänker tempgränsen extra under acceleration.",
                0f, 50f, 5f, 15f, ACCENT, 0.01f);

        addSectionTitle(root, "ADVANCED • VEHICLE / SI DATA");
        addAdvanced(root, "WHEEL", "Wheel diameter",
                "Hjuldiameter i mm för VESC speed/distance calculations.",
                200f, 300f, 1f, 254f, ACCENT, 1f);
        addAdvanced(root, "POLES", "Motor poles",
                "SI motor poles. Bara jämna värden skickas.",
                2f, 60f, 2f, 30f, ACCENT, 1f);
        addAdvanced(root, "GEAR", "Gear ratio",
                "SI gear ratio. Hub motor = normalt 1.0.",
                0.5f, 10f, 0.1f, 1f, ACCENT, 1f);
        addAdvanced(root, "CELLS", "Battery cells (series)",
                "SI battery cells. Stock G30 36 V pack = 10S.",
                8f, 14f, 1f, 10f, ACCENT, 1f);
        addAdvanced(root, "BATAH", "Battery capacity",
                "SI battery capacity i Ah för VESC display/range calculations.",
                5f, 30f, 0.5f, 15.3f, ACCENT, 1f);

        Button clearAdvancedButton = button("RENSA AVANCERADE ÄNDRINGAR", CARD_EDGE, TEXT);
        root.addView(clearAdvancedButton, fullButtonParams());
        clearAdvancedButton.setOnClickListener(v -> {
            for (AdvancedBinding binding : advancedBindings) binding.slider.markClean();
            Toast.makeText(this, "Avancerade ändringar rensade. Inget avancerat skickas.",
                    Toast.LENGTH_SHORT).show();
        });

        addSectionTitle(root, "APPLY & SAVE");
        LinearLayout warning = card();
        TextView warningTitle = text("STÅ HELT STILL", 16, ORANGE, true);
        warning.addView(warningTitle);
        warning.addView(text(
                "Gasreglaget ska vara släppt när du tillämpar eller sparar. " +
                        "Spara permanent skriver VESC-konfiguration och startar om ESP32.",
                13, MUTED, false
        ));
        root.addView(warning);

        applyButton = button("TILLÄMPA TILLFÄLLIGT", ACCENT, Color.rgb(2, 31, 27));
        saveButton = button("SPARA PERMANENT", ORANGE, Color.rgb(45, 25, 3));
        Button defaultsButton = button("SÄKRA STANDARDVÄRDEN", CARD_EDGE, TEXT);
        Button navigationButton = button("ÖPPNA NAVIGATION", CARD_EDGE, TEXT);

        root.addView(applyButton, fullButtonParams());
        root.addView(saveButton, fullButtonParams());
        root.addView(defaultsButton, fullButtonParams());
        root.addView(navigationButton, fullButtonParams());

        applyButton.setOnClickListener(v -> requestConfigAction(false));
        saveButton.setOnClickListener(v -> requestConfigAction(true));
        defaultsButton.setOnClickListener(v -> setSafeDefaults());
        navigationButton.setOnClickListener(v -> openNavigation());

        TextView footer = text(
                "FOC motor R/L/flux, current-controller KP/KI, hall/sensor detection, motor type, app selection och CAN setup är fortfarande låsta till VESC Tool.",
                12, MUTED, false
        );
        footer.setGravity(Gravity.CENTER);
        footer.setPadding(dp(10), dp(18), dp(10), 0);
        root.addView(footer);

        return scroll;
    }

    private void beginConnect() {
        if (!ble.isBluetoothSupported()) {
            showDialog("Bluetooth saknas", "Telefonen har inget BLE-stöd.");
            return;
        }

        if (!hasRequiredPermissions()) {
            requestRequiredPermissions();
            return;
        }

        if (!ble.isBluetoothEnabled()) {
            Intent enable = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enable, REQUEST_ENABLE_BT);
            return;
        }

        ble.scanAndConnect();
    }

    private boolean hasRequiredPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN)
                    == PackageManager.PERMISSION_GRANTED
                    && checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)
                    == PackageManager.PERMISSION_GRANTED;
        }
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED;
    }

    private void requestRequiredPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestPermissions(new String[]{
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT
            }, REQUEST_BT_PERMISSIONS);
        } else {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    REQUEST_BT_PERMISSIONS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String[] permissions,
                                           int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQUEST_BT_PERMISSIONS) return;

        boolean granted = grantResults.length > 0;
        for (int result : grantResults) {
            granted &= result == PackageManager.PERMISSION_GRANTED;
        }

        if (granted) {
            beginConnect();
        } else {
            new AlertDialog.Builder(this)
                    .setTitle("Bluetooth-behörighet behövs")
                    .setMessage("Tillåt Närliggande enheter för att ansluta till G30-NAV.")
                    .setPositiveButton("Öppna inställningar", (d, w) -> {
                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                        intent.setData(Uri.parse("package:" + getPackageName()));
                        startActivity(intent);
                    })
                    .setNegativeButton("Avbryt", null)
                    .show();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_OK) {
            ble.scanAndConnect();
        }
    }

    private void requestConfigAction(boolean permanent) {
        if (!connected || !ble.isReady()) {
            showDialog("Inte ansluten", "Anslut först till G30-NAV.");
            return;
        }

        String validation = validateSettings();
        if (validation != null) {
            showDialog("Kontrollera inställningarna", validation);
            return;
        }

        List<String> warnings = highRiskWarnings();
        Runnable send = () -> sendConfig(permanent);

        if (!warnings.isEmpty()) {
            StringBuilder message = new StringBuilder();
            for (String warning : warnings) {
                message.append("• ").append(warning).append('\n');
            }
            message.append("\nHöj stegvis och följ spänning och temperatur.");

            new AlertDialog.Builder(this)
                    .setTitle("Höga prestandavärden")
                    .setMessage(message.toString())
                    .setPositiveButton("Fortsätt", (d, w) -> send.run())
                    .setNegativeButton("Avbryt", null)
                    .show();
            return;
        }

        if (permanent) {
            new AlertDialog.Builder(this)
                    .setTitle("Spara permanent?")
                    .setMessage("Scootern måste stå helt still och gasen vara släppt. " +
                            "VESC-konfigurationen sparas och ESP32 startar om.")
                    .setPositiveButton("Spara", (d, w) -> send.run())
                    .setNegativeButton("Avbryt", null)
                    .show();
        } else {
            send.run();
        }
    }

    private void sendConfig(boolean permanent) {
        // Base G30/ESP settings are always applied first. Permanent save is
        // deliberately postponed until all changed Advanced VESC params have
        // been sent one-by-one.
        String command = String.format(Locale.US,
                "CFG|APPLY|%.1f|%.1f|%.1f|%.1f|%.1f|%d|%.1f|%d|%d|%.1f|%.1f|%.1f",
                driveCurrent.getValue(),
                fieldWeakening.getValue(),
                releaseRegen.getValue(),
                batteryCurrent.getValue(),
                batteryRegen.getValue(),
                policeSwitch.isChecked() ? 1 : 0,
                policeSpeed.getValue(),
                regenAbsSwitch.isChecked() ? 1 : 0,
                releaseBrakeSwitch.isChecked() ? 1 : 0,
                accelerationRamp.getValue(),
                cutoffStart.getValue(),
                cutoffEnd.getValue()
        );

        List<AdvancedBinding> dirtyAdvanced = new ArrayList<>();
        for (AdvancedBinding binding : advancedBindings) {
            if (binding.slider.isDirty()) dirtyAdvanced.add(binding);
        }

        connectionStatus.setText(permanent
                ? "Tillämpar och sparar " + dirtyAdvanced.size() + " avancerade värden …"
                : "Tillämpar " + dirtyAdvanced.size() + " avancerade värden …");
        connectionStatus.setTextColor(ORANGE);

        // Cancel callbacks from an older apply sequence.
        final int generation = ++advancedSequenceGeneration;
        actionHandler.removeCallbacksAndMessages(null);
        ble.sendLine(command);

        // VESC Lisp REPL is rate-limited. 800 ms spacing is intentionally
        // slower than the ESP32 firmware's 600 ms minimum.
        long delayMs = 800L;
        for (AdvancedBinding binding : dirtyAdvanced) {
            final AdvancedBinding b = binding;
            actionHandler.postDelayed(() -> {
                if (generation != advancedSequenceGeneration || !connected) return;
                ble.sendLine(String.format(Locale.US,
                        "ADV|SET|%s|%.4f", b.key, b.toFirmwareValue()));
            }, delayMs);
            delayMs += 800L;
        }

        if (permanent) {
            actionHandler.postDelayed(() -> {
                if (generation != advancedSequenceGeneration || !connected) return;
                ble.sendLine("ADV|SAVEALL");
            }, delayMs);
        } else if (!dirtyAdvanced.isEmpty()) {
            actionHandler.postDelayed(() -> {
                if (generation != advancedSequenceGeneration) return;
                connectionStatus.setText("Tillfälliga inställningar skickade");
                connectionStatus.setTextColor(GREEN);
            }, delayMs);
        }
    }

    private String validateSettings() {
        if (cutoffEnd.getValue() > cutoffStart.getValue() - 1f) {
            return "Voltage cutoff end måste ligga minst 1,0 V under cutoff start.";
        }

        AdvancedBinding vinMin = advancedByKey("VINMIN");
        AdvancedBinding vinMax = advancedByKey("VINMAX");
        if (vinMin != null && vinMax != null &&
                vinMin.slider.isDirty() && vinMax.slider.isDirty() &&
                vinMax.slider.getValue() <= vinMin.slider.getValue() + 2f) {
            return "Advanced VIN max bör ligga minst 2 V över VIN min.";
        }

        AdvancedBinding tStart = advancedByKey("TMSTART");
        AdvancedBinding tEnd = advancedByKey("TMEND");
        if (tStart != null && tEnd != null &&
                tStart.slider.isDirty() && tEnd.slider.isDirty() &&
                tEnd.slider.getValue() < tStart.slider.getValue() + 5f) {
            return "Motor temp cutoff end bör ligga minst 5 °C över start.";
        }
        return null;
    }

    private List<String> highRiskWarnings() {
        List<String> warnings = new ArrayList<>();
        if (fieldWeakening.getValue() > 10f) {
            warnings.add("Field weakening över 10 A kan ge mycket extra värme.");
        }
        if (batteryCurrent.getValue() > 20f) {
            warnings.add("Battery current över 20 A kan få original-G30-batteriets BMS att bryta.");
        }
        if (releaseRegen.getValue() > 8f) {
            warnings.add("Release e-brake över 8 A kan bromsa bakhjulet hårt.");
        }
        if (batteryRegen.getValue() > 5f) {
            warnings.add("Battery regen över 5 A är aggressivt för originalbatteriet.");
        }

        for (AdvancedBinding b : advancedBindings) {
            if (!b.slider.isDirty()) continue;
            if ("ERPM".equals(b.key) && b.slider.getValue() > 22000f)
                warnings.add("Max ERPM över 22 000 kan ge hög hjul-/motorhastighet.");
            if ("VMAX".equals(b.key) && b.slider.getValue() > 50f)
                warnings.add("VESC hard speed limit över 50 km/h kräver att hjul/poles/gear är korrekta.");
            if ("TMSTART".equals(b.key) && b.slider.getValue() > 85f)
                warnings.add("Motor temp cutoff start över 85 °C ger mindre termisk marginal.");
            if ("TMEND".equals(b.key) && b.slider.getValue() > 100f)
                warnings.add("Motor temp cutoff end över 100 °C är en hög temperaturgräns.");
            if ("DUTY".equals(b.key) && b.slider.getValue() > 92f)
                warnings.add("Maximum duty över 92 % ligger nära appens övre gräns.");
            if ("MINDUTY".equals(b.key) && b.slider.getValue() > 1f)
                warnings.add("Minimum duty är en låg-nivåinställning; höga värden kan ge konstigt lågfartsbeteende.");
        }
        return warnings;
    }

    private void setSafeDefaults() {
        driveCurrent.setValue(20f);
        fieldWeakening.setValue(0f);
        releaseRegen.setValue(2f);
        batteryCurrent.setValue(15f);
        batteryRegen.setValue(3f);
        policeSwitch.setChecked(false);
        policeSpeed.setValue(20f);
        regenAbsSwitch.setChecked(true);
        releaseBrakeSwitch.setChecked(true);
        accelerationRamp.setValue(60f);
        cutoffStart.setValue(34f);
        cutoffEnd.setValue(30f);
        for (AdvancedBinding binding : advancedBindings) binding.slider.markClean();
        Toast.makeText(this,
                "Säkra standardvärden är valda. Tryck Tillämpa eller Spara.",
                Toast.LENGTH_LONG).show();
    }

    private void openNavigation() {
        new AlertDialog.Builder(this)
                .setTitle("Öppna navigation")
                .setMessage("Bluetooth kan bara vara anslutet till en klient åt gången. " +
                        "Appen kopplar därför från G30-NAV innan kartan öppnas i Chrome.")
                .setPositiveButton("Öppna", (d, w) -> {
                    ble.disconnect();
                    setConnectedUi(false, "Frånkopplad för navigation");
                    startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(NAVIGATION_URL)));
                })
                .setNegativeButton("Avbryt", null)
                .show();
    }

    @Override
    public void onConnectionChanged(boolean isConnected, String message) {
        runOnUiThread(() -> setConnectedUi(isConnected, message));
    }

    @Override
    public void onLineReceived(String line) {
        runOnUiThread(() -> handleLine(line));
    }

    @Override
    public void onBleError(String message) {
        runOnUiThread(() -> {
            connectionStatus.setText(message);
            connectionStatus.setTextColor(RED);
            Toast.makeText(this, message, Toast.LENGTH_LONG).show();
        });
    }

    private void handleLine(String line) {
        if (line.startsWith("CFG|STATE|")) {
            Map<String, String> values = parseKeyValues(line, 2);
            setSlider(driveCurrent, values, "D");
            setSlider(fieldWeakening, values, "F");
            setSlider(releaseRegen, values, "R");
            setSlider(batteryCurrent, values, "BM");
            setSlider(batteryRegen, values, "BR");
            setSwitch(policeSwitch, values, "P");
            setSlider(policeSpeed, values, "PS");
            setSwitch(regenAbsSwitch, values, "A");
            setSwitch(releaseBrakeSwitch, values, "E");
            setSlider(accelerationRamp, values, "RA");
            setSlider(cutoffStart, values, "CS");
            setSlider(cutoffEnd, values, "CE");

            connectionStatus.setText("Inställningar lästa från ESP32");
            connectionStatus.setTextColor(GREEN);
            return;
        }

        if (line.startsWith("TEL|")) {
            updateTelemetry(parseKeyValues(line, 1));
            return;
        }

        if (line.startsWith("ACK|ADV_SET|")) {
            String key = line.substring("ACK|ADV_SET|".length()).trim();
            AdvancedBinding binding = advancedByKey(key);
            if (binding != null) binding.slider.markClean();
            connectionStatus.setText("Advanced " + key + " aktiv");
            connectionStatus.setTextColor(GREEN);
            return;
        }

        if (line.startsWith("ACK|ADV_SAVE")) {
            connectionStatus.setText("Allt sparat – ESP32 startar om …");
            connectionStatus.setTextColor(GREEN);
            for (AdvancedBinding binding : advancedBindings) binding.slider.markClean();
            Toast.makeText(this, "VESC + appinställningar sparade permanent.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        if (line.startsWith("ERR|ADV_BUSY")) {
            connectionStatus.setText("VESC upptagen – försök igen");
            connectionStatus.setTextColor(RED);
            Toast.makeText(this, "Ett avancerat kommando kom för tätt. Tryck Tillämpa igen.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        if (line.startsWith("ERR|ADV_VALUE") || line.startsWith("ERR|ADV_KEY")) {
            showDialog("Advanced-värde avvisat",
                    "ESP32:n avvisade ett avancerat VESC-värde utanför appens tillåtna område.");
            return;
        }

        if (line.startsWith("ACK|APPLY")) {
            connectionStatus.setText("Tillfälliga inställningar aktiva");
            connectionStatus.setTextColor(GREEN);
            Toast.makeText(this, "Inställningarna är aktiva tills nästa omstart.",
                    Toast.LENGTH_SHORT).show();
            return;
        }

        if (line.startsWith("ACK|SAVE")) {
            connectionStatus.setText("Sparat – ESP32 startar om …");
            connectionStatus.setTextColor(GREEN);
            Toast.makeText(this, "Sparat permanent. Anslut igen efter omstart.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        if (line.startsWith("ACK|PONG")) {
            connectionStatus.setText("G30-NAV svarar");
            connectionStatus.setTextColor(GREEN);
            return;
        }

        if (line.startsWith("ERR|STOP_FIRST") || line.startsWith("ERR|SAVE_BLOCKED")) {
            showDialog("Scootern måste stå still",
                    "Släpp gasen, låt hjulet stanna helt och försök igen.");
            return;
        }

        if (line.startsWith("ERR|INVALID_VALUE")) {
            showDialog("Värde avvisat",
                    "ESP32:n avvisade minst ett värde utanför den tillåtna säkerhetsgränsen.");
        }
    }

    private void updateTelemetry(Map<String, String> v) {
        speedValue.setText(value(v, "S", "--") + " km/h");
        voltageValue.setText(value(v, "V", "--") + " V");
        batteryCurrentValue.setText(value(v, "BIN", "--") + " A");
        motorCurrentValue.setText(value(v, "MOTOR", "--") + " A");
        motorTempValue.setText(value(v, "TM", "--") + " °C");
        vescTempValue.setText(value(v, "TV", "--") + " °C");

        String fault = value(v, "FAULT", "--");
        faultValue.setText(fault);
        faultValue.setTextColor("0".equals(fault) ? GREEN : RED);

        boolean armed = "1".equals(value(v, "ARMED", "0"));
        boolean braking = "1".equals(value(v, "EB", "0"));
        boolean police = "1".equals(value(v, "POLICE", "0"));

        String state = armed ? "Throttle armed" : "Throttle not armed";
        if (braking) state += " • E-brake";
        if (police) state += " • Police mode";
        driveStateValue.setText(state);
        driveStateValue.setTextColor(armed ? GREEN : ORANGE);
    }

    private Map<String, String> parseKeyValues(String line, int start) {
        Map<String, String> map = new HashMap<>();
        String[] parts = line.split("\\|");
        for (int i = start; i < parts.length; i++) {
            int equals = parts[i].indexOf('=');
            if (equals <= 0) continue;
            map.put(parts[i].substring(0, equals), parts[i].substring(equals + 1));
        }
        return map;
    }

    private static String value(Map<String, String> map, String key, String fallback) {
        String value = map.get(key);
        return value == null || value.isEmpty() ? fallback : value;
    }

    private void setSlider(SettingSlider slider, Map<String, String> values, String key) {
        String raw = values.get(key);
        if (raw == null) return;
        try {
            slider.setValue(Float.parseFloat(raw));
        } catch (NumberFormatException ignored) {
        }
    }

    private void setSwitch(Switch widget, Map<String, String> values, String key) {
        String raw = values.get(key);
        if (raw != null) widget.setChecked("1".equals(raw));
    }

    private AdvancedBinding addAdvanced(LinearLayout root,
                                        String key,
                                        String title,
                                        String subtitle,
                                        float min,
                                        float max,
                                        float step,
                                        float initial,
                                        int color,
                                        float firmwareScale) {
        SettingSlider slider = new SettingSlider(title, subtitle, min, max, step, initial, color);
        slider.markClean();
        AdvancedBinding binding = new AdvancedBinding(key, slider, firmwareScale);
        advancedBindings.add(binding);
        root.addView(slider.view);
        return binding;
    }

    private AdvancedBinding advancedByKey(String key) {
        for (AdvancedBinding binding : advancedBindings) {
            if (binding.key.equals(key)) return binding;
        }
        return null;
    }

    private void setConnectedUi(boolean isConnected, String message) {
        connected = isConnected;
        if (!isConnected) {
            advancedSequenceGeneration++;
            actionHandler.removeCallbacksAndMessages(null);
        }
        connectionStatus.setText(message);
        connectionStatus.setTextColor(isConnected ? GREEN : ORANGE);
        connectButton.setText(isConnected ? "KOPPLA FRÅN" : "ANSLUT");
        readButton.setEnabled(isConnected);
        applyButton.setEnabled(isConnected);
        saveButton.setEnabled(isConnected);
        readButton.setAlpha(isConnected ? 1f : 0.45f);
        applyButton.setAlpha(isConnected ? 1f : 0.45f);
        saveButton.setAlpha(isConnected ? 1f : 0.45f);
    }

    private void showDialog(String title, String message) {
        new AlertDialog.Builder(this)
                .setTitle(title)
                .setMessage(message)
                .setPositiveButton("OK", null)
                .show();
    }

    private LinearLayout card() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(dp(14), dp(13), dp(14), dp(13));
        card.setBackground(rounded(CARD, CARD_EDGE, 14));
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
        p.bottomMargin = dp(10);
        card.setLayoutParams(p);
        return card;
    }

    private LinearLayout horizontal() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        return row;
    }

    private LinearLayout.LayoutParams weighted() {
        return new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
    }

    private LinearLayout.LayoutParams fullButtonParams() {
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(52)
        );
        p.bottomMargin = dp(9);
        return p;
    }

    private TextView text(String value, float size, int color, boolean bold) {
        TextView view = new TextView(this);
        view.setText(value);
        view.setTextSize(size);
        view.setTextColor(color);
        view.setTypeface(Typeface.create("sans", bold ? Typeface.BOLD : Typeface.NORMAL));
        view.setLineSpacing(0, 1.06f);
        return view;
    }

    private TextView telemetryBig(LinearLayout parent,
                                  String label,
                                  String initial,
                                  int color) {
        TextView caption = text(label, 12, MUTED, true);
        caption.setLetterSpacing(0.08f);
        parent.addView(caption);

        TextView value = text(initial, 38, color, true);
        value.setPadding(0, 0, 0, dp(8));
        parent.addView(value);
        return value;
    }

    private TextView stat(LinearLayout row, String label, String initial, int color) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        box.setPadding(dp(7), dp(8), dp(7), dp(8));
        box.setGravity(Gravity.CENTER);
        box.setBackground(rounded(Color.rgb(11, 19, 24), CARD_EDGE, 8));

        TextView caption = text(label, 10, MUTED, true);
        caption.setGravity(Gravity.CENTER);
        box.addView(caption);

        TextView value = text(initial, 16, color, true);
        value.setGravity(Gravity.CENTER);
        value.setPadding(0, dp(3), 0, 0);
        box.addView(value);

        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        p.setMargins(dp(3), dp(3), dp(3), dp(3));
        row.addView(box, p);
        return value;
    }

    private void addSectionTitle(LinearLayout root, String title) {
        TextView view = text(title, 13, ACCENT, true);
        view.setLetterSpacing(0.12f);
        view.setPadding(dp(3), dp(9), 0, dp(8));
        root.addView(view);
    }

    private Switch addToggle(LinearLayout root,
                             String title,
                             String subtitle,
                             boolean checked) {
        LinearLayout card = card();
        LinearLayout row = horizontal();

        LinearLayout labels = new LinearLayout(this);
        labels.setOrientation(LinearLayout.VERTICAL);
        labels.addView(text(title, 16, TEXT, true));
        labels.addView(text(subtitle, 12, MUTED, false));
        row.addView(labels, new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f));

        Switch widget = new Switch(this);
        widget.setChecked(checked);
        row.addView(widget);
        card.addView(row);
        root.addView(card);
        return widget;
    }

    private Button button(String label, int background, int foreground) {
        Button button = new Button(this);
        button.setText(label);
        button.setTextSize(12);
        button.setTextColor(foreground);
        button.setTypeface(Typeface.DEFAULT_BOLD);
        button.setAllCaps(false);
        button.setGravity(Gravity.CENTER);
        button.setPadding(dp(9), 0, dp(9), 0);
        button.setBackground(rounded(background, background, 10));
        return button;
    }

    private Space space(int width) {
        Space space = new Space(this);
        space.setLayoutParams(new LinearLayout.LayoutParams(width, 1));
        return space;
    }

    private GradientDrawable rounded(int fill, int stroke, int radiusDp) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fill);
        drawable.setCornerRadius(dp(radiusDp));
        drawable.setStroke(dp(1), stroke);
        return drawable;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    @Override
    protected void onDestroy() {
        ble.disconnect();
        super.onDestroy();
    }

    private final class AdvancedBinding {
        final String key;
        final SettingSlider slider;
        final float firmwareScale;

        AdvancedBinding(String key, SettingSlider slider, float firmwareScale) {
            this.key = key;
            this.slider = slider;
            this.firmwareScale = firmwareScale;
        }

        float toFirmwareValue() {
            return slider.getValue() * firmwareScale;
        }
    }

    private final class SettingSlider {
        final LinearLayout view;
        final SeekBar seek;
        final TextView valueText;
        final float min;
        final float max;
        final float step;
        boolean dirty;

        SettingSlider(String title,
                      String subtitle,
                      float min,
                      float max,
                      float step,
                      float initial,
                      int color) {
            this.min = min;
            this.max = max;
            this.step = step;

            view = card();

            LinearLayout top = horizontal();
            LinearLayout labels = new LinearLayout(MainActivity.this);
            labels.setOrientation(LinearLayout.VERTICAL);
            labels.addView(text(title, 16, TEXT, true));
            labels.addView(text(subtitle, 12, MUTED, false));
            top.addView(labels, new LinearLayout.LayoutParams(0,
                    LinearLayout.LayoutParams.WRAP_CONTENT, 1f));

            valueText = text("", 17, color, true);
            valueText.setGravity(Gravity.END);
            valueText.setPadding(dp(8), 0, 0, 0);
            top.addView(valueText);
            view.addView(top);

            seek = new SeekBar(MainActivity.this);
            seek.setMax(Math.round((max - min) / step));
            seek.setProgressTintList(android.content.res.ColorStateList.valueOf(color));
            seek.setThumbTintList(android.content.res.ColorStateList.valueOf(TEXT));
            LinearLayout.LayoutParams sliderParams = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT
            );
            sliderParams.topMargin = dp(8);
            view.addView(seek, sliderParams);

            LinearLayout limits = horizontal();
            TextView minLabel = text(format(min), 11, MUTED, false);
            TextView maxLabel = text(format(max), 11, MUTED, false);
            maxLabel.setGravity(Gravity.END);
            limits.addView(minLabel, weighted());
            limits.addView(maxLabel, weighted());
            view.addView(limits);

            seek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    if (fromUser) dirty = true;
                    refreshValue();
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {
                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {
                }
            });

            setValue(initial);
        }

        float getValue() {
            return min + seek.getProgress() * step;
        }

        void setValue(float value) {
            float clamped = Math.max(min, Math.min(max, value));
            seek.setProgress(Math.round((clamped - min) / step));
            dirty = false;
            refreshValue();
        }

        boolean isDirty() {
            return dirty;
        }

        void markClean() {
            dirty = false;
        }

        private void refreshValue() {
            valueText.setText(format(getValue()));
        }

        private String format(float value) {
            if (step >= 1f) return String.format(Locale.US, "%.0f", value);
            return String.format(Locale.US, "%.1f", value);
        }
    }
}
