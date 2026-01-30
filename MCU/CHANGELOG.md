# MCU Firmware Changelog

All notable changes to the Bimo Robotics Kit MCU firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Serial availability checks before reading a specific amount of data.
- Total system voltage reading (useful for battery-powered implementations).
- Total servo current draw reading from switch IC.
- General serial communications robustness improvements.

---

## [0.9.5] - 2026-01-30

### Added
- Versioned firmware header:
  - Added file header documenting firmware version (`0.9.5-Beta`), target MCU (`RP2040`), and license (Apache-2.0).

- Structured state data:
  - Introduced a `StateData` struct grouping all MCU telemetry into a single 112-byte block:
    - IMU quaternion (4 floats).
    - Distance sensor readings (4 × `uint16_t`).
    - Servo feedback:
      - Positions (`uint16_t[8]`).
      - Speeds (`int16_t[8]`).
      - Loads (`int16_t[8]`).
      - Voltages (`uint16_t[8]`).
      - Currents (`int16_t[8]`).
      - Temperatures (`uint8_t[8]`).

- Dual-core distance sensing:
  - I2C multiplexer (`TCA9548A`) support for four VL53L0X distance sensors.
  - New `core1Entry()` loop running on the second RP2040 core to:
    - Continuously poll all four VL53L0X distance sensors at ~33 ms.
    - Store readings in a `distBuffer[4]` intermediate array.
    - Copy values into `state.dist[]` with a simple handshake (`core0Reading` / `core1Writing`) to avoid race conditions and keep the main loop running at ~4 ms.

- Servo safety limits:
  - Added per-servo minimum and maximum limits:
    - `servoMin[8]` and `servoMax[8]` arrays.
  - Enforced limits in `applyNewPositions()` before commanding the actuators.

### Changed
- State data protocol:
  - Old version (`v0.5`) packed multiple arrays manually into an 80-byte buffer:
    - IMU quaternion, distances, battery, positions.
    - Battery and distance values were dummy placeholders; battery support will be added in a future update.
  - New version (`v0.9.5`) sends the entire `StateData` struct in one `Serial.write((uint8_t*)&state, sizeof(state));`.
  - This simplifies parsing on the host side and allows adding more feedback data easily.


- IMU output:
  - IMU now only outputs the orientation quaternion.
  - Gyroscope data extraction remains in the code but is commented out for baseline models.

- Command handling:
  - Removed message 4 (request for current servo positions); these are now included in the `StateData` struct.
  - `msg == 1` now returns the full `StateData` struct instead of a custom 80-byte buffer.
  - `msg == 2` remains the alive/ready response.
  - `msg == 3` is now the calibration request (servo offset calibration for all 8 servos).

- Servo feedback collection:
  - Old code only used `currPositions[8]` as feedback and did not expose speed, load, voltage, current, or temperature in a structured way.
  - New code adds `updateServoFeedback()` which:
    - Calls `servoDriver.FeedBack(i)` for each servo.
    - Fills all `state.pos[]`, `state.speed[]`, `state.load[]`, `state.voltage[]`, `state.current[]`, and `state.temp[]` without repeated driver calls.
  - `servoDriver.WritePosEx()` now uses a speed limit set to 3400. This saturates the servo speed, providing maximum speed at any time.


---

## [0.5.0] - 2025-12-29

### Added
- Initial public MCU firmware for the Bimo Robotics Kit:
  - Basic IMU integration using `Adafruit_BNO08x`:
    - Quaternion-based orientation via `SH2_GYRO_INTEGRATED_RV`.
    - Gyroscope data from the same `SH2_GYRO_INTEGRATED_RV` report.
  - Servo control via `SMS_STS` driver over `Serial1`.

- State reporting:
  - Packed IMU quaternion, distance readings (dummy), battery level (dummy), and servo positions into a single 80-byte buffer and sent over USB serial.

- Basic command protocol over serial:
  - `msg == 1`: Send state data.
  - `msg == 2`: Alive/ready status.
  - `msg == 3`: Send current positions.
  - `msg == 4`: Run servo offset calibration on all 8 servos.

> Note: This release corresponds to the original MCU firmware baseline for Bimo, before structured state data and dual-core distance processing were introduced.
