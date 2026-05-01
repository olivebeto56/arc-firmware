# running_node — Sprint 1 firmware

Firmware del nodo de tobillo para el prototipo de running. Un único sketch `.ino`
que se flashea **idéntico** en las dos bandas. Cada banda genera en boot su
propio nombre BLE `SportBand-XXXX` a partir del chip ID único del nRF52840
(`NRF_FICR->DEVICEID`); la asignación de pie izquierdo/derecho la hace la app
durante el pairing.

## Hardware

- **MCU:** Seeed XIAO nRF52840 Sense
- **IMU:** Adafruit BNO085 breakout (#4754) — I2C @ `0x4A`, SDA=D4, SCL=D5
- **Batería:** LiPo 3.7 V 400 mAh (JST 1.25 mm)

## Setup en Arduino IDE (una vez)

1. Preferences → *Additional Boards Manager URLs*:
   ```
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   ```
2. Boards Manager → instalar **Seeed nRF52 mbed-enabled Boards**.
3. Library Manager → instalar:
   - `Adafruit BNO08x` (arrastra como dependencias `Adafruit BusIO` y `Adafruit Unified Sensor`)
   - `ArduinoBLE` ≥ 1.3.0
4. Tools → Board → **Seeed XIAO BLE Sense - nRF52840**.

## Flashear los dos nodos

El sketch es **idéntico** para las dos bandas — no hay nada que editar entre
uploads. Cada banda genera su propio nombre BLE en boot a partir del chip ID
único del nRF52840 (formato `SportBand-XXXX`, 4 hex chars).

1. Conecta el primer XIAO por USB-C → *Tools → Board → Seeed XIAO BLE Sense -
   nRF52840* → Upload.
2. Verifica en el Serial Monitor (115200 bps):
   ```
   [main] AI Sport Monitor — running node booting
   [sensor] BNO085 ready (9-axis ARVR + linear accel + calibrated gyro)
   [ble] advertising as SportBand-A3F2
   [main] ready
   ```
3. Apunta el sufijo (`A3F2` en el ejemplo) y márcalo físicamente en la banda
   con rotulador o pegatina. **Hazlo ahora** — luego, dentro de la carcasa,
   no hay forma de distinguirlas a simple vista sin reconectar por USB.
4. Desconecta y repite con la segunda banda. Tendrá un sufijo distinto
   (probabilidad de colisión ≈ 1/65536).

> La asignación de pie izquierdo/derecho se hace en la app durante el pairing
> ("agita la del tobillo izquierdo"). El firmware no conoce su lado.

## Comprobar con un escáner BLE

Antes de tener la app Flutter lista, usa **nRF Connect** (Android/iOS):

- Buscas dos peripherals con nombres `SportBand-XXXX` y `SportBand-YYYY`
  (cada banda tiene su sufijo único).
- Servicio `19B10000-…`, tres caracteristicas:
  - `19B10001` (NOTIFY) — al suscribirte deberías ver paquetes de 22 bytes a 100 Hz.
  - `19B10002` (READ/NOTIFY) — un byte con `%` de batería.
  - `19B10003` (WRITE) — reservado para configurar sample rate.

## Formato del paquete sensor (v3, 22 bytes, little-endian)

| Offset | Tipo  | Campo            | Escala                          |
|-------:|-------|------------------|---------------------------------|
| 0–1    | u16   | timestamp_ms     | ms desde inicio sesión (wraps ~65 s) |
| 2–3    | i16   | qw               | × `QUAT_SCALE` (10000)          |
| 4–5    | i16   | qx               | × `QUAT_SCALE` (10000)          |
| 6–7    | i16   | qy               | × `QUAT_SCALE` (10000)          |
| 8–9    | i16   | qz               | × `QUAT_SCALE` (10000)          |
| 10–11  | i16   | accel_x          | milli-g                         |
| 12–13  | i16   | accel_y          | milli-g                         |
| 14–15  | i16   | accel_z          | milli-g                         |
| 16–17  | i16   | gyro_x           | × `GYRO_SCALE` (°/s)            |
| 18–19  | i16   | gyro_y           | × `GYRO_SCALE` (°/s)            |
| 20–21  | i16   | gyro_z           | × `GYRO_SCALE` (°/s)            |

> En la app, accel se reconvierte a m/s² con `× 9.80665 / 1000`, y gyro a °/s
> dividiendo entre `GYRO_SCALE`.

`GYRO_SCALE` está definido en `config.h`. Por defecto vale **100** (rango
±327 °/s, running/gym). Para análisis de golf se baja a **10** (rango
±3270 °/s, swing en muñeca) — mantener firmware y app en sync.

### Compatibilidad con versiones anteriores (parser-side)

El firmware emite siempre v3. Los parsers cliente deben degradar por
`bytes.length` y nunca lanzar excepción por packets cortos:

| Versión | Tamaño | Contenido                            |
|--------:|-------:|--------------------------------------|
| v1      | 14 B   | timestamp + quat + accel_x/y (sin accel_z, sin gyro) |
| v2      | 16 B   | timestamp + quat + accel completo (sin gyro) |
| v3      | 22 B   | v2 + gyro_x/y/z (formato actual)     |

## Decisiones de diseño

- **9 ejes (ARVR_STABILIZED_RV) — nunca `GAME_ROTATION_VECTOR`.** El magnetómetro es
  necesario para tracking estable de yaw. Sin él, en 30 s el dato deriva.
- **Abstracción `getSensorAngles()`.** El BLE no sabe que hay un BNO085 detrás —
  cuando migremos a PCB propio, sólo se reemplaza `sensor.cpp`.
- **Identidad runtime (chip ID).** El firmware es idéntico en las dos bandas;
  el sufijo del nombre BLE viene del `NRF_FICR->DEVICEID` factory-burned. La
  asignación de pie/lado vive en la app, no en el firmware — esto generaliza
  a multi-deporte (golf con muñeca + cadera, gym con muñeca + tobillo, etc.).
- **Re-advertising automático en disconnect.** El nodo nunca queda “muerto” tras
  perder la conexión — vuelve a anunciarse de inmediato.
- **Reset del timestamp al reconectar.** La app detecta saltos en `timestamp_ms`
  y reinicia su buffer de eventos.

## Próximo paso

Sprint 2 — Flutter app: `BleManager` que se conecte a las dos bandas en paralelo
con `Future.wait()` y parsee este paquete en `SensorParser`.
