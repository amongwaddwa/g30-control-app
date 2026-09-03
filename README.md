# G30 Control – Androidapp för ESP32 + UBOX

Det här projektet innehåller en native Androidapp och matchande ESP32-firmware.
Appen ansluter via Bluetooth LE till `G30-NAV` och kan läsa livevärden samt
ändra ett avgränsat urval av kör- och batteriinställningar.

## Funktioner

- Live: hastighet, batterispänning, batteriström, motorström, motor-/VESC-temp,
  fault, throttle-armed, e-brake och Police mode.
- Drive current: 5–35 A.
- Field weakening: 0–35 A.
- Release e-brake: 0–12 A.
- Battery current max: 5–25 A.
- Battery regen max: 0–8 A.
- Police mode + gräns 10–25 km/h.
- Regen ABS på/av.
- E-brake vid släppt gas på/av.
- Acceleration response: 20–120 A/s.
- Battery cutoff start/end.
- Tillämpa tillfälligt eller spara permanent.
- Knapp som kopplar från och öppnar den befintliga G30-navigationen i Chrome.

Appen är medvetet **inte** en full ersättare för VESC Tool. Motor detection,
hall/FOC setup, firmware, temperaturgivartyp och hårdvaruspecifika parametrar
ska fortfarande hanteras i officiella VESC Tool.

## 1. Flasha rätt ESP32-firmware

Använd filen:

`firmware/SimpleVescDisplay_G30_UART_Throttle_V19_ANDROID_APP.ino`

Bluetooth-enheten ska därefter heta `G30-NAV`.

Eftersom firmwaren innehåller Bluetooth och grafik behöver CYD-projektet normalt
partitionen **Huge APP / 3 MB APP** i Arduino IDE.

## 2. Bygg APK automatiskt på GitHub

Projektet innehåller `.github/workflows/build-apk.yml`.

1. Skapa ett nytt publikt eller privat GitHub-repository.
2. Ladda upp **innehållet** i den här projektmappen till repositoryts rot.
3. Öppna fliken **Actions**.
4. Öppna **Build G30 Control APK**.
5. Tryck **Run workflow** om bygget inte redan startade automatiskt.
6. När bygget är grönt: öppna körningen och ladda ner artifacten
   **G30-Control-APK**.
7. Packa upp artifacten och installera `app-debug.apk` på Android.

Android kan fråga om tillåtelse att installera appar från webbläsaren eller
filhanteraren. Tillåt det endast för den app du använder för installationen.

## 3. Använd appen

1. Starta ESP32/UBOX.
2. Öppna G30 Control.
3. Tryck **ANSLUT** och godkänn *Närliggande enheter*.
4. Appen söker i högst 10 sekunder och ansluter till `G30-NAV`.
5. Tryck **LÄS INSTÄLLNINGAR** om värdena inte laddas automatiskt.
6. Stå helt still och släpp gasen innan du trycker **TILLÄMPA** eller
   **SPARA PERMANENT**.

`TILLÄMPA TILLFÄLLIGT` ändrar den aktiva konfigurationen men överlever inte alla
omstarter. `SPARA PERMANENT` sparar ESP32-inställningarna, kör VESC `conf-store`
och startar om ESP32.

## Säkra startvärden för original G30-batteri

- Drive current: 20 A
- Field weakening: 0 A
- Release e-brake: 2 A
- Battery current max: 15 A
- Battery regen max: 3 A
- Police mode: av
- Police speed: 20 km/h
- Regen ABS: på
- Release e-brake: på
- Acceleration response: 60 A/s
- Cutoff start/end: 34 / 30 V

Höj en sak i taget. Om spänningen sjunker kraftigt eller hela scootern stängs av,
sluta testa och kontrollera batteri, BMS, kontakter och VESC-gränser.

## Navigation och app samtidigt

ESP32 accepterar normalt en BLE-klient åt gången. Därför kopplar appens
navigationsknapp från G30 Control innan GitHub Pages-navigationen öppnas.
