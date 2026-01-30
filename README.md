# MotorNodeESP32S3 — Closed-Loop Stepper Controller (ESP32-S3 + TMC2209 + AS5047P)

Firmware for an **ESP32-S3** based stepper “motor node” on a **custom PCB**:
- **TMC2209** stepper driver (UART config + STEP/DIR motion)
- **AS5047P** absolute magnetic encoder (SPI) for closed-loop monitoring (step-loss detect + recovery nudges)
- **Thermistor** temperature monitoring (ADC + beta model)
- **ESP-NOW** control/telemetry protocol (CRC16 + versioned binary frames)
- **FastAccelStepper** motion engine for reliable high-rate pulse generation on ESP32

This project is built in **Arduino IDE** but uses multiple `.cpp` files arranged in subfolders.

---

## Features

### Motion control
- Enable / disable motor
- Stop motion
- Absolute target move (degrees * 100)
- Velocity mode (deg/s * 100)
- Encoder-based homing to a target angle window + settle time
- Runtime-configurable:
  - microsteps
  - max speed (rev/s * 100)
  - acceleration (rev/s² * 100)

### Closed-loop monitoring (step-loss)
- Compares commanded position vs encoder position in **FULL STEPS**
- Configurable thresholds for hold/run/fast speed bands
- Confirmation timers to avoid false positives
- Recovery nudges when loss is active
- Fault on recovery timeout (auto-disable for a configurable time)

### Comms (ESP-NOW)
- Binary protocol with:
  - header magic/version
  - CRC16-CCITT validation
  - ACK responses
  - telemetry status frames
  - TLV-based configuration get/set

### Telemetry
- Motor state, enabled/moving flags
- Position/target/home in steps
- Encoder angle (wrapped + unwrapped)
- Step error estimate
- Fault code
- Temperature (°C * 10)
- Config revision

---

## Hardware assumptions

### MCU
- ESP32-S3 module (Arduino core for ESP32)

### Stepper driver: TMC2209
- STEP/DIR from ESP32
- UART (Serial1) for configuration
- **Custom PCB current setting:**
  - **15k resistor between 5VOUT and VREF**
  - **No external current sense resistors**
  - **BRA and BRB tied to GND**
  - Firmware must enable **internal RDSon sensing** (`internal_Rsense(true)`) in `GCONF`

> ⚠️ Important: If you are using “no sense resistors” mode, you must configure the driver accordingly, otherwise current regulation may be incorrect/unstable.

### Encoder: AS5047P
- SPI connection (CS pin configurable)
- Firmware unwraps angle across multiple turns by detecting wrap jumps

### Thermistor
- Divider assumed: **3.3V → R_fixed → ADC → NTC → GND**
- Beta model conversion (configurable R25, Beta, R_fixed)

---

## Repository structure

- `MotorNodeESP32S3.ino` — Arduino entry point; initializes everything and starts tasks
- `BuildAll.cpp` — forces Arduino to compile `.cpp` files in subfolders (Arduino build quirk)

### Key modules
- `drivers/TMC2209Driver.*` — TMC2209 UART + register config wrapper
- `sensors/EncoderAS5047P.*` — SPI read + unwrap logic for AS5047P
- `sensors/Thermistor.*` — ADC sampling + beta conversion
- `motion/MotionController.*` — high-level state machine (enable/move/velocity/home/fault)
- `motion/StepLossMonitor.*` — step-loss detection and recovery logic
- `motion_engine/MotionEngine_FAS.*` — FastAccelStepper-backed motion engine
- `comms/Protocol.*` — wire protocol structs + CRC16
- `comms/EspNowManager.*` — ESP-NOW init + RX parsing + dispatch + telemetry TX
- `config/ConfigDefs.h` — config IDs + defaults
- `config/ConfigStore.*` — TLV apply + NVS persistence
- `system/Tasks.*` — FreeRTOS tasks pinned to cores
- `system/PinMap.h` — single source of truth for GPIOs
- `system/SharedState.h` — thread-safe telemetry snapshot

---

## Pin map (default)

See `system/PinMap.h`:

### Stepper
- `PIN_STEP = 10`
- `PIN_DIR  = 11`
- `PIN_EN   = 47` (**active-high** enable on this board)

### TMC2209 UART (Serial1)
- `PIN_UART_TX = 7`
- `PIN_UART_RX = 16` (through 1k resistor)

### Encoder SPI
- `SCK  = 13`
- `MOSI = 14`
- `MISO = 12`
- `CS   = 21`

### Thermistor ADC
- `PIN_THERM_ADC = 8`

### LEDs
- `PIN_LED_MOVE  = 36`
- `PIN_LED_COMMS = 35`

---

## Build & flash (Arduino IDE)

1. Install **ESP32 Arduino core** (Boards Manager)
2. Install libraries:
   - **FastAccelStepper**
   - **TMCStepper**
3. Open `MotorNodeESP32S3.ino`
4. Select the correct ESP32-S3 board + COM port
5. Build and upload

> Note: `BuildAll.cpp` is used so Arduino links `.cpp` files located in subfolders. Keep it.

---

## Runtime architecture (FreeRTOS tasks)

Tasks are started in `system/Tasks.cpp`:

- **Core 1**: `MotionTask` (time-critical)
  - runs `MotionController::tick()` at ~500 Hz
- **Core 0**: `SensorTask`
  - thermistor sampling at configurable interval
- **Core 0**: `TelemetryTask`
  - periodic ESP-NOW status frames
- **Core 0**: `LedTask`
  - move LED patterns + RX blink indicator

---

## Protocol overview (ESP-NOW)

Defined in `comms/Protocol.h`:
- Header with magic/version/CRC16
- Message types:
  - `CMD_PING`
  - `CMD_ENABLE`
  - `CMD_STOP`
  - `CMD_MOVE_ABS_DEG` (deg*100)
  - `CMD_VELOCITY_DPS` (deg/s*100)
  - `CMD_HOME_START`
  - `CMD_GET_CONFIG` / `CMD_SET_CONFIG` / `CMD_SAVE_CONFIG`
- Responses:
  - `RSP_ACK`
  - `RSP_STATUS`
  - `RSP_CONFIG`

### Config TLV format
Each entry:
- `param_id (u16)`
- `type (u8)`
- `len (u8)`
- `value[len]`

Config is stored in NVS namespace `motorcfg`.

---

## Configuration parameters

Defined in `config/ConfigDefs.h` (stable IDs):
- Microsteps, max speed, acceleration
- TMC current settings: `IRUN`, `IHOLD`
- Homing parameters
- Step-loss thresholds and timings
- Thermistor parameters
- Telemetry interval

---

## Notes / known limitations

- Encoder SPI read uses a simple “command then NOP” two-frame read pattern. If your AS5047P variant uses different flags/mode, adjust in `EncoderAS5047P.cpp`.
- Step-loss compares in **full steps**, because encoder resolution/accuracy makes microstep-level comparison misleading.
- Current setup depends on your PCB mode (RREF to VREF and BRA/BRB to GND). If you switch to external sense resistors, you must update the TMC configuration accordingly.

---

## Safety

This firmware can move motors with significant torque.
- Test with low current first
- Keep clear of moving mechanisms
- Add an emergency stop / power cutoff in hardware where appropriate

---

## License

Add your preferred license here (MIT / Apache-2.0 / GPL / proprietary).
