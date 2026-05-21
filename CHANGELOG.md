# Changelog

All notable changes to the Bimo Robotics Kit software will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased] - 1.1.0

### BimoAPI
#### Fixed
- `Bimo.calibrate()`: corrected ankle target pose to use negative angle values,
  fixing servos not reaching the physical calibration position.

#### Changed
- `bimo.py`: `self.servo_min` and `self.servo_max` shoulder joint limits tightened from ±12° to ±8°
  to improve walking stability.
- `api_example.py`: observation vector updated to 11 elements
  (3 IMU orientation + 8 last actions), matching the Isaac Lab task environment.

### MCU
#### Added
- RP2040 internal temperature (`rp_temp`) exposed in `StateData` struct and
  available via `state["rp_temp"]` in the Python API. Struct size is now 118 bytes.

#### Changed
- Distance sensor init now retries up to 3 times on startup before continuing silently.