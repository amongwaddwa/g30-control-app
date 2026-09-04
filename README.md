# G30 Control v2 – Androidapp för ESP32 + UBOX

Det här projektet innehåller Androidappen och matchande ESP32-firmware. Appen ansluter via BLE till `G30-NAV`.

## Vanliga inställningar

- Drive current 5–35 A
- Field weakening 0–35 A
- Release e-brake 0–12 A
- Battery current max 5–25 A
- Battery regen max 0–8 A
- Police mode + 10–25 km/h
- Regen ABS
- Release e-brake on/off
- Acceleration/current ramp 20–120 A/s
- Battery cutoff start/end

## Advanced VESC

Version 2 lägger dessutom till 20 avancerade reglage:

- Motor current scale
- Brake current scale
- Maximum wattage
- Maximum brake wattage
- Maximum duty cycle
- Minimum duty cycle
- Maximum ERPM
- ERPM limit start
- VESC hard speed limit
- Field weakening duty start
- Minimum input voltage
- Maximum input voltage
- Motor temp cutoff start
- Motor temp cutoff end
- Acceleration temperature decrease
- Wheel diameter
- Motor poles
- Gear ratio
- Battery cells in series
- Battery capacity Ah

**Viktigt:** Advanced-reglagen läses inte automatiskt från VESC. De visas med startvärden i appen, men ett avancerat värde skickas **endast om du själv flyttar det reglaget**. Det är medvetet för att appen inte ska skriva över en befintlig VESC-konfiguration av misstag.

Motor detection, FOC motor R/L/flux, current-controller KP/KI, hall/sensor mode, motor type, app selection och CAN-konfiguration är fortfarande inte exponerade i appen. De kan göra controllern eller motorn obrukbar om de blir fel och bör göras i VESC Tool.

## Firmware

Flasha:

`firmware/SimpleVescDisplay_G30_UART_Throttle_V26_MOBILE_ADVANCED_APP.ino`

Den behåller den klassiska/originala SimpleVescDisplay-huvudskärmen, Android BLE, touchmenyer och navigation.

Använd normalt **Huge APP / 3 MB APP** på ESP32-CYD om standardpartitionen är för liten.

## Bygg APK på GitHub

1. Skapa ett GitHub-repository.
2. Ladda upp innehållet i projektmappen till repositoryts rot.
3. Öppna **Actions**.
4. Kör **Build G30 Control APK**.
5. Ladda ner artifacten `G30-Control-APK`.
6. Packa upp och installera `app-debug.apk`.

## Apply / Save

Scootern ska stå still och gasen vara släppt.

`TILLÄMPA TILLFÄLLIGT` skickar basinställningarna och därefter endast de Advanced-reglage du faktiskt har ändrat.

`SPARA PERMANENT` gör samma sak, väntar mellan avancerade kommandon och avslutar med VESC `conf-store`, sparar ESP32-inställningarna och startar om ESP32.
