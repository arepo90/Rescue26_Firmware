# CLAUDE.md — Rescue_Firmware

Embedded firmware for the ESP32 (DOIT DevKit V1), Arduino framework, PlatformIO.
Companion software lives in `../Rescue_Software/` (ROS2 Humble, Python, C++).

---

## Build & Flash

```bash
pio run                                   # build
pio run --target upload                   # flash
pio run --target upload && pio device monitor  # flash + monitor
pio run --target clean                    # clean artifacts
pio device monitor                        # serial monitor (115200 baud)
pio test                                  # on-device unit tests
```

Build environment: `[env:esp32doit-devkit-v1]`
Add libraries under `lib_deps` in `platformio.ini`.
Project `include/` is exposed to library builds via `-I"${PROJECT_INCLUDE_DIR}"` in `build_flags`.

---

## Architecture

### Module layout (18 source files, 7 libraries)

```
include/
  config.h          — all pin defs, timing constants, FreeRTOS task config
  robot_types.h     — shared structs (PPMFrame, EncoderState, sensor data,
                       protocol payloads). Includes config.h.
lib/
  RC/               — GPIO ISR decoder (RISING-edge timing, sync gap > 3000 µs)
  Encoders/         — ESP32 PCNT hardware (4× quadrature, 32-bit overflow ISR)
  Locomotion/       — LEDC servo-PWM, tank-drive mixing
  Sensors/          — LIS3MDL + MLX90640 + MQ2; standby until mini PC enables
  CANInterface/     — MCP2515 SPI wrapper; arm protocol STUBBED (see below)
  Comms/            — Binary framed UART protocol, RX state machine, callbacks
  Control/          — Top-level state machine, ESTOP, PPM failsafe
src/main.cpp        — FreeRTOS task creation only; loop() does nothing
```

### Dual-core FreeRTOS task split

| Core | Task | Priority | Period |
|------|------|----------|--------|
| 0 | `commsTask` — UART RX/TX + 50 Hz telemetry | 4 | 1 ms poll |
| 0 | `canTask` — MCP2515 RX poll | 4 | 5 ms |
| 1 | `controlTask` — PPM + state machine | 5 (highest) | 20 ms (50 Hz) |
| 1 | `sensorTask` — background sensor reads | 2 (lowest) | continuous |

`loop()` calls `vTaskDelay(portMAX_DELAY)` — it is never used.

### State machine (Control)

```
INIT → STANDBY → MAIN ⇄ ARM
            ↑               ↓  (ESTOP from mini PC)
            └──── ESTOP ────┘  (cleared only by mini PC MSG_ESTOP_CLEAR)
```

- **MAIN**: Ch2 (forward/back) + Ch4 (turn) → tank-drive mixing → LEDC PWM.
  Ch1 → flipper angle target.
- **ARM**: tracks stopped. Ch1-4,6 sent to mini PC for IK via `Comms`.
  Arm joint angles received back → forwarded over CAN.
- **PPM failsafe**: if no valid frame for 500 ms → STANDBY + neutralise motors.

---

## Key Hardware (ROBOT_MAIN)

| Function | GPIO |
|----------|------|
| I2C SDA / SCL | 21 / 22 |
| SPI MOSI / MISO / SCK | 23 / 19 / 18 |
| CAN CS (MCP2515) | 5 |
| UART2 TX / RX (mini PC) | 17 / 16 |
| PPM input | 34 (input-only) |
| Motor PWM — Left / Right / Flipper | 25 / 26 / 27 |
| Encoder Left A / B | 32 / 33 |
| Encoder Right A / B | 35 / 36 (input-only) |
| Encoder Flipper A / B | 13 / 14 |
| MQ2 analog | 39 (input-only) |

LEDC: 50 Hz, 14-bit resolution. PCNT: 4× quadrature decode, hardware overflow ISR.

---

## ESP32 ↔ Mini PC Binary Protocol

**Frame**: `[0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]`
**CRC**: XOR of TYPE + LEN_H + LEN_L + all PAYLOAD bytes.
**UART2**: 921600 baud, 8N1, TX=17 RX=16.

### ESP32 → PC

| Type | Name | Rate | Payload |
|------|------|------|---------|
| `0x01` | Telemetry | 50 Hz | `TelemetryPayload` (see robot_types.h): mode(u8), flags(u8), ppm[6](u16), speed_L/R(i16 ×10 RPM), flipper_angle(i16 ×10 °), uptime_ms(u32) |
| `0x02` | Thermal | sensor rate | `ThermalPayload`: 768 × i16 (°C×10) + i16 ambient |
| `0x03` | Mag | sensor rate | `MagPayload`: x/y/z_uT×100 (i16×3), heading_deg×10 (i16) |
| `0x04` | Gas | sensor rate | `GasPayload`: rs_ro×100, ppm_lpg, ppm_co, ppm_smoke (i16×4) |
| `0x05` | Status | on change | mode(u8), flags(u8), sensor_mask(u8), pad(u8) |

### PC → ESP32

| Type | Name | Payload |
|------|------|---------|
| `0x10` | Arm joints | `ArmJointsPayload`: joint[6] (i16, degrees×100) |
| `0x11` | Sensor enable | 1 byte bitmask: bit0=mag, bit1=thermal, bit2=gas |
| `0x12` | ESTOP | 0 bytes — immediate stop |
| `0x13` | ESTOP clear | 0 bytes — resume from STANDBY |

All sensors default to **standby** (mask=0) until the mini PC sends `0x11`.

---

## Stubbed / Needs Context

| Item | Location | What's needed |
|------|----------|---------------|
| CAN arm protocol | `lib/CANInterface/CANInterface.cpp` | Arbitration IDs, data layout for joint commands |
| Encoder CPR | `include/config.h` `ENC_CPR_TRACK/FLIPPER` | Counts-per-rev from motor/encoder datasheet |
| Gear ratios | `include/config.h` `TRACK/FLIPPER_GEAR_RATIO` | Motor-to-output reduction ratios |
| MQ2 Ro calibration | `include/config.h` `MQ2_RO_KOHM` | Bench measurement in clean air |

---

## Design Conventions

- All high-level functions are robot-agnostic. Hardware differences between
  ROBOT_MAIN and ROBOT_SECONDARY are isolated in `apply*()` private methods
  and gated with `#ifdef ROBOT_MAIN / ROBOT_SECONDARY`.
- Mutexes (`portMUX_TYPE`) live as file-level statics in `.cpp` files, never
  in class declarations, to keep FreeRTOS headers out of `.h` files.
- `IRAM_ATTR` is placed only on function **definitions** (`.cpp`), not declarations.
- No WiFi, no Bluetooth. All inter-device communication is wired.
- The DOIT DevKit V1 always shows `E (133) psram: PSRAM ID read error` and
  double-boots on power-on — both are normal, not firmware crashes.