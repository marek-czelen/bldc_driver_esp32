# ⚡ BLDC Motor Driver for ESP32 - Real-Time Multi-Mode E-Bike Controller

An embedded firmware project for driving a three-phase BLDC motor from an ESP32-based controller. It converts Hall sensors, throttle, pedal-assist, brake, current, battery-voltage, and S866 display inputs into deterministic motor-control output through a 20 kHz MCPWM interrupt path.

The project is aimed at e-bike and small electric-vehicle control experiments, with a practical progression from six-step commutation to sinusoidal control and FOC, plus serial and local Wi-Fi diagnostics.

[![C++](https://img.shields.io/badge/C%2B%2B-Embedded-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-F5822A?logo=platformio&logoColor=white)](https://platformio.org/)
[![Arduino](https://img.shields.io/badge/Framework-Arduino-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/Target-ESP32-E7352C?logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32)
[![Wi-Fi](https://img.shields.io/badge/Connectivity-Wi--Fi-2196F3?logo=wifi&logoColor=white)](https://www.wi-fi.org/)

## 🧰 Used Technologies

| Area | Technology | Evidence in the repository |
| --- | --- | --- |
| Firmware | C++ with the Arduino framework | `.cpp` sources, Arduino APIs, `framework = arduino` |
| Build | PlatformIO | `platformio.ini`, `esp32dev` environment |
| Target | ESP32 development board | `board = esp32dev`, ESP32 GPIO/ADC/MCPWM registers |
| Motor PWM | ESP32 MCPWM, center-aligned `UP_DOWN` mode | `src/main.cpp`, `include/pinout.h` |
| Motor control | Hall-synchronized BLOCK, SINUS, FOC, and multi-step commutation | `include/bldc_types.h`, `src/main.cpp` |
| Persistence | ESP32 NVS through Arduino `Preferences` | `src/bldc_config.cpp` |
| Connectivity | Built-in `WiFi`, `WebServer`, and `DNSServer` | `src/main.cpp` |
| Display protocol | S866 protocol 2 over `Serial2` | `src/display_s866.cpp`, `include/display_s866.h` |
| Diagnostics | Serial command interface and embedded HTML/JavaScript dashboard | `src/main.cpp` |

The PlatformIO configuration does not pin an explicit `espressif32` platform version. The exact framework and toolchain versions therefore come from the PlatformIO package resolution on the development machine.

## 🚀 Key Features

### Motor control

- Six-step trapezoidal commutation based on the three Hall sensors.
- `BLOCK12` half-step commutation between Hall transitions.
- `BLOCK24` runtime mode, presented in the UI and command interface as `BLOCK12BL`, with duty blending between commutation vectors.
- Sinusoidal commutation using a 96-sample electrical-angle lookup table plus a guard entry.
- Continuous Q16 angle tracking with Hall correction, sector-speed tracking, phase-offset tuning, and stall/startup handling.
- FOC control with Clarke/Park transforms, d/q PI regulators, inverse transforms, and SVPWM generation.
- FOC voltage mode for diagnostics and a relay-feedback PI auto-tuning command.
- Software direction reversal for CW/CCW operation.

### Real-time control path

- MCPWM unit 0 with three operators, one for each motor phase.
- Center-aligned PWM with a default frequency of 20 kHz and a configurable 8-32 kHz range.
- Direct low-level MCPWM register writes from an IRAM-safe TEZ ISR, avoiding driver API spinlocks in the time-critical path.
- Hall validation with invalid-state rejection, consecutive-sample confirmation, and minimum-period filtering.
- Current ADC sampling at the PWM valley, fixed-point EMA filtering, and high-sample rejection for phase-current spikes.

### Inputs, assistance, and braking

- Throttle input filtering using a rolling sample buffer, median selection, and outlier rejection.
- PAS sampling through an `esp_timer` callback, configurable debounce, cadence measurement, and direction detection from signal asymmetry.
- PAS start delay, soft-start ramp, speed-target PI control, freewheel hysteresis, and slew-rate limiting.
- S866 drive-mode selection for PAS + throttle, throttle-only, or PAS-only operation.
- Active-low brake input plus a serial brake simulation command.
- Regenerative braking using low-side PWM, with minimum-RPM and battery-voltage cutoffs and an 80% maximum regen duty.
- Global speed limiting with a filtered power fade and configurable fallback values when the display is absent.

### Configuration and diagnostics

- Binary controller configuration stored in ESP32 NVS and validated with a magic value and configuration version.
- S866 protocol 2 support with XOR checksum validation, frame resynchronization, connection confirmation, and telemetry replies.
- 921600-baud main serial console and 9600-baud S866 `Serial2` link.
- Local Wi-Fi access point with an embedded responsive dashboard, configuration JSON, telemetry JSON, command execution, and captive DNS redirection.
- Diagnostics for Hall transitions, sinusoidal tracking, current readings, commutation events, FOC state, speed sensing, PAS, and individual MOSFET-side testing.

## 🧱 Hardware and Pinout

The firmware is written for an ESP32 development board connected to three IR2103 half-bridge gate drivers, one per motor phase. The IR2103 inputs use active-high HIN and active-low LIN logic; the code keeps this polarity explicit in the phase helper functions.

### Power stage and measurements

| Function | Hardware described by the code | ESP32 pins |
| --- | --- | --- |
| Phase A gate control | IR2103 HIN/LIN | GPIO12 / GPIO13 |
| Phase B gate control | IR2103 HIN/LIN | GPIO25 / GPIO26 |
| Phase C gate control | IR2103 HIN/LIN | GPIO27 / GPIO14 |
| Phase-current feedback | INA180A2, gain 50 V/V, 2 mOhm shunts | GPIO39 / GPIO34 / GPIO35 |
| Battery voltage | Resistor divider, values calibrated in code | GPIO36 |
| Throttle | Analog input, calibrated raw range 400-2600 | GPIO33 |
| FET temperature input | ADC input declared in the pin map | GPIO32 |

The current-sense channels are ADC1 channels. The firmware reads them at the MCPWM TEZ point, filters them in the ISR, and reconstructs signed phase currents for SINUS/FOC calculations using the three-phase current relationship.

### Sensors and user interfaces

| Function | Pin or interface | Notes |
| --- | --- | --- |
| Hall A / B / C | GPIO4 / GPIO18 / GPIO19 | `INPUT_PULLUP`, three-bit CBA Hall state |
| Brake | GPIO23 | Active low, `INPUT_PULLUP` |
| PAS | GPIO22 | Timer-sampled and digitally filtered |
| External speed sensor | GPIO21 | Falling-edge input for geared/direct external sensing mode |
| S866 RX / TX | GPIO16 / GPIO17 | `Serial2`, 9600 baud, 8N1 |
| Level-shifter enable | GPIO5 | Enables the TXB0102DCU path used by the display interface |
| Main console | Board USB-UART `Serial` | 921600 baud |

GPIO12 is an ESP32 strap pin and is also used for phase-A high-side control. The hardware must avoid an inappropriate pull-up on this pin during boot, otherwise flash programming can fail.

## 🏗️ Architecture

```text
platformio.ini
├── include/
│   ├── pinout.h          Hardware pins and MCPWM timing constants
│   ├── bldc_types.h      Drive modes, phase states, and runtime state
│   ├── bldc_config.h     NVS configuration layout and public API
│   └── display_s866.h    S866 protocol types and service API
├── src/
│   ├── main.cpp          Setup, control loop, MCPWM ISR, control modes,
│   │                      sensors, serial commands, Wi-Fi API, diagnostics
│   ├── bldc_config.cpp   Preferences/NVS load, validation, defaults, save
│   └── display_s866.cpp  S866 frame parsing, checksum, and telemetry TX
└── SINUS_POWER_FIX_PLAN.md
                       Engineering notes for SINUS/FOC power investigations
```

The firmware has four primary execution paths:

1. `setup()` initializes NVS, GPIO, ADC1, PAS sampling, MCPWM, the safe all-MOSFET-off state, the 20 kHz commutation timer, S866, and the Wi-Fi dashboard.
2. `loop()` reads and filters slower inputs, calculates throttle/PAS targets, applies ramps and limits, runs the FOC current loop when selected, services S866 and HTTP clients, processes serial commands, and emits diagnostics.
3. `onCommutationTimer()` runs on the MCPWM TEZ event. It samples phase current and Hall inputs, validates Hall transitions, updates speed/angle state, then dispatches BLOCK, SINUS, FOC, or regenerative commutation through direct MCPWM register writes.
4. Telemetry is sent to the S866 display, the serial console, and the local HTTP dashboard from the non-ISR path.

## 🧩 Core Modules

| Module | Responsibility |
| --- | --- |
| [`src/main.cpp`](src/main.cpp) | Complete runtime orchestration: initialization, sensor acquisition, throttle/PAS logic, Hall processing, block/sinus/FOC/regen commutation, current limiting, serial commands, Wi-Fi server, dashboard data, and diagnostics. |
| [`include/bldc_types.h`](include/bldc_types.h) | `drive_mode_t`, phase-state definitions, Hall state constants, and the shared `bldc_state_t` runtime model. |
| [`include/pinout.h`](include/pinout.h) | ESP32 GPIO assignment, MCPWM resolution/period, default PWM frequency, duty range, and dead-time constants. |
| [`include/bldc_config.h`](include/bldc_config.h) | Packed `controller_config_t`, `CONFIG_MAGIC`, `CONFIG_VERSION`, and NVS API declarations. |
| [`src/bldc_config.cpp`](src/bldc_config.cpp) | Loads the binary configuration from the `bldc/cfg` NVS entry, applies defaults when validation fails, and persists updates. |
| [`include/display_s866.h`](include/display_s866.h) | S866 frame sizes, protocol fields, display configuration model, and service interface. |
| [`src/display_s866.cpp`](src/display_s866.cpp) | Polling UART service, XOR checksum validation, sliding-window frame resynchronization, connection timeout handling, and 14-byte responses. |
| [`SINUS_POWER_FIX_PLAN.md`](SINUS_POWER_FIX_PLAN.md) | Current engineering notes and test hypotheses for SINUS/FOC power behavior; it is not a production validation report. |

## 🔄 Data Flow / API Overview

### Control and telemetry flow

| Step | Component | Responsibility |
| --- | --- | --- |
| 1 | `config_init()` | Load and validate the 70-byte NVS configuration, or create defaults. |
| 2 | `loop()` input path | Read battery voltage, phase current snapshots, throttle, Hall state, brake, PAS, and display data. |
| 3 | Control calculation | Combine throttle/PAS demand, apply ramps, speed/current limits, regen conditions, and calculate FOC d/q voltage targets when applicable. |
| 4 | `onCommutationTimer()` | Execute the deterministic 20 kHz TEZ path and update the three MCPWM operators for the active control mode. |
| 5 | Output interfaces | Send S866 telemetry, serial diagnostics, and JSON telemetry to the local web dashboard. |

### HTTP API

The ESP32 starts an open local access point named `BLDC_Config` and serves HTTP on `192.168.4.1:80`. The HTML dashboard is compiled into the firmware.

| Method | Endpoint | Description |
| --- | --- | --- |
| `GET` | `/` | Embedded control and diagnostics dashboard. |
| `GET` | `/api/config` | Current NVS-backed configuration, assist override, and queued command field. |
| `GET` | `/api/telemetry` | Current mode, speed, power, duty, Hall/PAS state, currents, FOC values, display data, and fault state. |
| `GET`, `POST` | `/api/cmd` | Execute a serial-style command. The command is supplied as the `cmd` argument or form body. |
| `POST` | `/api/queue` | Store a command string in the in-memory queued-command field. The current code exposes the field but does not contain a consumer that executes it later. |
| Any other path | Redirect | Captive-portal requests are redirected to `/`. |

## 🛠️ Getting Started

### Requirements

- An ESP32 board compatible with the PlatformIO `esp32dev` board definition.
- PlatformIO Core or the PlatformIO IDE extension for VS Code.
- A USB connection capable of uploading firmware and opening a serial monitor.
- A correctly wired three-phase power stage, IR2103 gate drivers, Hall sensors, current-sense circuits, throttle/brake/PAS inputs, and suitable power protection.
- An S866 display is expected by the default configuration (`display_required = 1`). Standalone operation can be enabled through the firmware command interface and fallback settings.

No external database, cloud service, package manager, or environment variables are required by the repository configuration.

### Build

From the repository root:

```bash
pio run -e esp32dev
```

The project uses the single environment declared in [`platformio.ini`](platformio.ini):

- Platform: `espressif32`
- Board: `esp32dev`
- Framework: `arduino`
- Partition layout: `default.csv`
- Build flags: `CORE_DEBUG_LEVEL=3` and `CONFIG_ARDUHAL_LOG_COLORS=1`

### Upload and monitor

```bash
pio run -e esp32dev -t upload
pio device monitor -b 921600
```

The main console uses 921600 baud. The S866 display uses an independent `Serial2` connection at 9600 baud.

On first boot, the firmware initializes NVS defaults when the `bldc/cfg` blob is missing or does not match `CONFIG_MAGIC`, `CONFIG_VERSION`, or the expected 70-byte layout. The default PWM frequency is 20 kHz, the default ramp is 1200 ms, and the default phase-current limit is 15 A unless the connected S866 P14 value overrides it.

### First serial checks

With the board powered and the motor power stage secured for bench testing, useful commands include:

| Command | Purpose |
| --- | --- |
| `h` | Print the command help. |
| `s` | Print a one-shot runtime status snapshot. |
| `B` | Select BLOCK commutation. |
| `B12` | Select `BLOCK12`. |
| `B12BL` | Select `BLOCK12BL`. |
| `S` | Select SINUS commutation. |
| `F` | Select FOC. |
| `d` | Disable the motor and force all MOSFETs off. |
| `a` | Toggle periodic auto-status output. |
| `R` | Toggle regenerative braking. |
| `gdbg`, `fdbg`, `idbg`, `hdbg`, `cdbg` | Toggle focused diagnostics. |
| `t`, `tAH`, `tAL`, `tBH`, `tBL`, `tCH`, `tCL` | Enter or inspect individual MOSFET-side diagnostics. |

The command parser also exposes configuration, PAS, speed-calibration, FOC-tuning, phase-offset, assist, and PWM-frequency commands. See the command parsing section in [`src/main.cpp`](src/main.cpp) for the authoritative implementation.

## 🔐 Security Features / Security Considerations

### Implemented safeguards

- Invalid Hall states are rejected, and valid transitions require consecutive confirmation plus a minimum time interval.
- The brake input forces the motor demand down and can be simulated for diagnostics through serial.
- Phase-current limiting combines a proportional duty reduction with a hard over-current lockout after repeated readings above the configured threshold.
- Regenerative braking is disabled above the 42 V battery threshold, below the minimum effective RPM, and above the configured 80% regen duty ceiling.
- The startup path explicitly drives all MOSFETs off before enabling the commutation timer.
- S866 frames use XOR checksum validation, a 50 ms inter-byte timeout, sliding-window resynchronization, and a connection timeout.
- NVS configuration is checked with a magic value, version, and exact packed-structure size before use.

### Important limitations

- The Wi-Fi AP and HTTP API have no authentication, authorization, TLS, or request-rate limiting. `/api/cmd` can execute serial-style motor and configuration commands.
- The AP credentials are hardcoded in `src/main.cpp` as `BLDC_Config` / `bldc1234`. This is suitable only for a controlled bench network; credentials and access control must be redesigned before production use.
- The FET temperature pin is declared in the pin map, but the current analog-input path does not actively sample it. There is no implemented thermal shutdown.
- S866 P15 undervoltage data is parsed and exposed, but the firmware does not implement a corresponding undervoltage cutoff.
- NVS validation uses magic, version, and size; it does not provide a CRC or authenticated integrity check.
- The project does not contain a watchdog-based recovery strategy for a stalled control task.
- Power-stage protection remains a hardware responsibility. Gate-driver supply, bootstrap design, MOSFET ratings, current-sense placement, transient suppression, and oscilloscope validation are not replaceable by firmware limits.

Do not connect an unverified power stage to a motor or battery solely because the firmware builds. Start with a current-limited supply, a mechanically secured motor, and independent validation of gate signals, phase nodes, current readings, and regenerative-voltage behavior.

## 📌 Project Status & Future Improvements

**Status: working embedded-control prototype / portfolio project.** The repository contains a substantial hardware-oriented implementation, including real-time MCPWM commutation, multiple control modes, persistent configuration, display protocol handling, and diagnostics. It should not be described as production-ready: the repository has no automated test suite, CI pipeline, release packaging, or production security model.

The following improvements are directly motivated by the current code and documentation:

- Add host-side unit tests for Hall-to-phase tables, commutation transitions, speed calculations, NVS validation, current limiting, and FOC transforms.
- Add hardware-in-the-loop or bench regression procedures for startup, Hall faults, brake response, current limiting, and regen cutoff behavior.
- Implement a real thermal measurement and cutoff path, plus an enforced undervoltage policy.
- Add authenticated configuration access and remove the hardcoded Wi-Fi password from production builds.
- Add a watchdog and explicit fault-latching/recovery behavior for control-task failures.
- Pin the PlatformIO platform/framework versions for reproducible builds.
- Split the large runtime implementation in `src/main.cpp` into independently testable control, I/O, protocol, and web modules.
- Resolve the repository-wide licensing question before redistribution. There is no `LICENSE` file, while [`src/display_s866.cpp`](src/display_s866.cpp) documents adaptation from EBiCS_Firmware under GPL v3.
- Reconcile the firmware version printed at boot (`1.0.0`, build marker `ADC_SYNC_BLKSTART`) with the version history and release metadata.

## 📚 Engineering Notes

[`SINUS_POWER_FIX_PLAN.md`](SINUS_POWER_FIX_PLAN.md) records investigation notes for SINUS/FOC power behavior and should be read as an engineering work log rather than a guarantee of a validated fix. The current source contains synchronized phase-current acquisition in the MCPWM ISR, but FOC and high-power SINUS operation still require validation on the actual motor, gate-driver circuit, and power stage.

## 🎓 Portfolio Project

> Portfolio Project - This firmware demonstrates embedded C++ development for ESP32, real-time motor-control scheduling, ISR-safe peripheral access, sensor filtering, persistent configuration, device protocols, and practical safety-oriented diagnostics.
