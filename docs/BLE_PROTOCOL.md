# G30 Control BLE-protokoll v2

ESP32 annonserar som `G30-NAV` och använder Nordic-UART-liknande UUID:er:

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- Telefon skriver till RX: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- Telefon tar emot notiser från TX: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

## Basinställningar

```text
CFG|GET
CFG|PING
CFG|RESTART
CFG|APPLY|drive|fw|releaseRegen|batteryMax|batteryRegen|police|policeSpeed|regenAbs|releaseEBrake|ramp|cutStart|cutEnd
```

App v2 använder alltid `CFG|APPLY` först. Permanent save görs sist med `ADV|SAVEALL`, så alla avancerade kommandon hinner appliceras innan `conf-store`.

## Advanced VESC

Ett värde per BLE-paket:

```text
ADV|SET|KEY|VALUE
ADV|SAVEALL
```

Nycklar:

```text
CURS     -> l-current-max-scale
BRS      -> l-current-min-scale
MINDUTY  -> l-min-duty
DUTY     -> l-max-duty
WMAX     -> l-watt-max
WBRAKE   -> l-watt-min (appen skickar positiv W, ESP32 gör värdet negativt)
ERPM     -> l-max-erpm
ERPMS    -> l-erpm-start
VINMIN   -> l-min-vin
VINMAX   -> l-max-vin
TMSTART  -> l-temp-motor-start
TMEND    -> l-temp-motor-end
TACC     -> l-temp-accel-dec
FWDUTY   -> foc-fw-duty-start
VMAX     -> max-speed (appen använder km/h, ESP32 konverterar till m/s)
WHEEL    -> si-wheel-diameter (appen använder mm, ESP32 konverterar till meter)
BATAH    -> si-battery-ah
CELLS    -> si-battery-cells
POLES    -> si-motor-poles
GEAR     -> si-gear-ratio
```

Avancerade kommandon skickas med ca 800 ms mellanrum för att respektera VESC Lisp REPL-rate-limit.

## ESP32 till app

```text
CFG|STATE|D=25.0|F=5.0|R=3.0|BM=15.0|BR=3.0|P=0|PS=20.0|A=1|E=1|RA=60.0|CS=34.0|CE=30.0
TEL|S=22.4|V=38.5|BIN=8.2|MOTOR=19.4|TM=41|TV=38|FAULT=0|ARMED=1|EB=0|POLICE=0
ACK|APPLY
ACK|ADV_SET|ERPM
ACK|ADV_SAVE|RESTARTING
ERR|STOP_FIRST
ERR|INVALID_VALUE
ERR|ADV_BUSY
ERR|ADV_VALUE
```
