# MagicCube
**Multi-Sensor Embedded Platform — v2.1**

Compact ESP32-S3 sensor node exposing a real-time web dashboard over a self-hosted Wi-Fi access point. Designed for rapid deployment in robotics prototyping, environment characterisation, and motion-capture applications.

---

## Platform

| Parameter | Value |
|---|---|
| MCU | ESP32-S3 Dev Module |
| I2C Bus | SDA GPIO 8 / SCL GPIO 9 / 400 kHz |
| Access Point | SSID `MagicCube`, open, IP `192.168.4.1` |
| Poll Rate | 50 ms |
| Power | 3.3 V rail throughout |

---

## Sensor Suite

| Sensor | Bus Address | Measurements |
|---|---|---|
| BME688 | `0x76` | Temperature, Humidity, Pressure, Gas Resistance, Altitude (derived), Dew Point (derived), Heat Index (derived) |
| BNO055 | `0x28` | Accelerometer XYZ (m/s2), Gyroscope XYZ (deg/s), Magnetometer XYZ (uT), Euler RPY (deg), Quaternion WXYZ |
| VL53L1X | `0x29` | Distance (mm), Velocity (m/s, EMA-filtered) |
| TSL2561 | `0x39` | Illuminance (lux) |

---

## Electrical Interface

All sensors share a single I2C bus. The table below details every pin connection required for correct operation. Do not leave address-select or mode-select pins floating.

### Common Bus

| Signal | ESP32-S3 GPIO |
|---|---|
| SDA | 8 |
| SCL | 9 |
| XSHUT (VL53L1X only) | 3 |

---

### BME688

Mode select: I2C is active when CS is held HIGH. SDO sets the 7-bit address LSB.

| BME688 Pin | Connection | Note |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| SDI | GPIO 8 | I2C data |
| SCK | GPIO 9 | I2C clock |
| SDO | GND | Address LSB = 0, resolves to `0x76` |
| CS | 3.3 V | Must be HIGH to enable I2C mode |

> Pull SDO to 3.3 V to shift address to `0x77`. CS floating or LOW disables I2C.

---

### BNO055

PS0 and PS1 select the serial protocol. ADR sets the 7-bit address LSB.

| BNO055 Pin | Connection | Note |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| SDA | GPIO 8 | I2C data |
| SCL | GPIO 9 | I2C clock |
| PS0 | GND | Protocol select — I2C mode |
| PS1 | GND | Protocol select — I2C mode |
| ADR | 3.3 V | Address LSB = 1, resolves to `0x28` |
| NRST | 3.3 V or float | Active-low reset, leave unconnected if unused |

> Pull ADR to GND to shift address to `0x29`. External crystal is enabled in firmware via `setExtCrystalUse(true)`.

---

### VL53L1X

XSHUT is active-low. The firmware asserts GPIO 3 HIGH before I2C initialisation to bring the device out of hardware standby.

| VL53L1X Pin | Connection | Note |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| SDA | GPIO 8 | I2C data |
| SCL | GPIO 9 | I2C clock |
| XSHUT | GPIO 3 | Driven HIGH in firmware; pull LOW to reset |
| GPIO1 | Not connected | Interrupt output, unused |

> Configured for Long distance mode, 20 ms timing budget, continuous ranging. Maximum range: 4 m.

---

### TSL2561

ADDR pin selects between three fixed I2C addresses.

| TSL2561 Pin | Connection | Note |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| SDA | GPIO 8 | I2C data |
| SCL | GPIO 9 | I2C clock |
| ADDR | Leave floating | Resolves to `0x39` |
| INT | Not connected | Interrupt output, unused |

> Connecting ADDR to GND shifts address to `0x29` (conflicts with VL53L1X). Connecting to 3.3 V shifts to `0x49`. Leave floating for `0x39`.

---

## Derived Quantities

### Altitude
Derived from barometric pressure using the International Standard Atmosphere:

```
altitude = 44330 * (1 - (P / 1013.25) ^ 0.1903)    [metres]
```

### Dew Point
Magnus approximation:

```
alpha = (17.27 * T) / (237.3 + T) + ln(RH / 100)
T_dew = (237.3 * alpha) / (17.27 - alpha)            [degC]
```

### Heat Index
NWS regression (Steadman), computed in degF internally, returned in degC.

### Velocity
Differentiated from successive VL53L1X range measurements. Three-stage conditioning pipeline:

| Stage | Parameter | Value |
|---|---|---|
| Minimum time delta | MIN_DT | 20 ms |
| Noise gate (single-step jump) | MAX_VEL_JUMP | 3.0 m/s |
| Hard clamp | MAX_VEL_MS | +/- 5.0 m/s |
| EMA smoothing factor | VEL_ALPHA | 0.25 |

```
v_raw  = delta_distance_mm / delta_time_s / 1000.0    [m/s]
v_filt = 0.25 * v_raw + 0.75 * v_prev                 [m/s, EMA]
```

---

## Firmware Dependencies

Install via Arduino IDE Library Manager:

| Library | Author |
|---|---|
| Adafruit BME680 Library | Adafruit |
| Adafruit BNO055 | Adafruit |
| Adafruit Unified Sensor | Adafruit |
| VL53L1X | Pololu |
| Adafruit TSL2561 Unified | Adafruit |

Additional: `Wire`, `WiFi`, `WebServer` — bundled with ESP32 Arduino core.

---

## Build Configuration

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| Partition Scheme | Default 4 MB |
| PSRAM | Disabled |

---

## Deployment

1. Flash firmware to the ESP32-S3.
2. Connect to Wi-Fi access point `MagicCube` (no passphrase).
3. Navigate to `http://192.168.4.1` in any browser.
4. Sensor status indicators in the dashboard header confirm successful initialisation.

Serial monitor at 115200 baud reports per-sensor initialisation status on startup.

---

## Troubleshooting

| Symptom | Probable Cause | Corrective Action |
|---|---|---|
| BME688 not detected | CS not tied HIGH | Connect CS to 3.3 V |
| BME688 address mismatch | SDO state undefined | Confirm SDO to GND for `0x76` |
| BNO055 not detected | PS0/PS1 selecting SPI | Confirm both PS0 and PS1 to GND |
| VL53L1X not detected | XSHUT held LOW | Confirm GPIO 3 wired to XSHUT; verify GPIO is initialised before `Wire.begin()` |
| TSL2561 address conflict | ADDR tied to GND | Leave ADDR floating for `0x39` |
| I2C bus lockup | Missing pull-up resistors | Add 4.7 kohm pull-ups on SDA and SCL to 3.3 V if not on breakout board |
| Velocity unstable | Mechanical vibration or fast motion | Decrease VEL_ALPHA toward 0.1 for heavier smoothing |
| Dashboard unreachable | Client not associated to AP | Confirm Wi-Fi association before navigating to `192.168.4.1` |

---

