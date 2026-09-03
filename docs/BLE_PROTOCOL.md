# G30 Control BLE-protokoll

ESP32 annonserar som `G30-NAV` och använder Nordic-UART-liknande UUID:er:

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- Telefon skriver till RX: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- Telefon tar emot notiser från TX: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

## App till ESP32

```text
CFG|GET
CFG|PING
CFG|RESTART
CFG|APPLY|drive|fw|releaseRegen|batteryMax|batteryRegen|police|policeSpeed|regenAbs|releaseEBrake|ramp|cutStart|cutEnd
CFG|SAVE|drive|fw|releaseRegen|batteryMax|batteryRegen|police|policeSpeed|regenAbs|releaseEBrake|ramp|cutStart|cutEnd
```

Exempel:

```text
CFG|APPLY|25.0|5.0|3.0|15.0|3.0|0|20.0|1|1|60.0|34.0|30.0
```

## ESP32 till app

```text
CFG|STATE|D=25.0|F=5.0|R=3.0|BM=15.0|BR=3.0|P=0|PS=20.0|A=1|E=1|RA=60.0|CS=34.0|CE=30.0
TEL|S=22.4|V=38.5|BIN=8.2|MOTOR=19.4|TM=41|TV=38|FAULT=0|ARMED=1|EB=0|POLICE=0
ACK|APPLY
ACK|SAVE|RESTARTING
ERR|STOP_FIRST
ERR|INVALID_VALUE
```
