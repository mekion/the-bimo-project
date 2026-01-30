# Changelog

All notable changes to the Bimo Robotics Kit Python API will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- State data additions:
  - Total servo current draw reading from switch IC on PCB.
  - Voltage reading from main power source on PCB.

- Automatic `stand_up` routine when robot falls down, regardeless of end orientation.


## [0.9.5] - 2026-01-30

### Added
- New `api_example.py` showing end-to-end usage:
  - Model loading with ONNX runtime.
  - State observation processing.
  - Action post-processing and command streaming to the robot.
  - Example camera usage.

- Camera handling:
  - Explicit detection of two USB cameras (front and top).
  - Resolution validation with a fixed set of supported resolutions.
  - Explicit MJPEG configuration using `cv2.VideoWriter_fourcc('M','J','P','G')`.
  - Helper `capture_image()` to grab a single BGR frame by name ("front" or "top").

- New state structure which includes the following data as a Python `dict`:
  - Orientation in Euler angles.
  - Distances from distance sensor readings.
  - Servo positions.
  - Servo speeds.
  - Servo loads.
  - Servo voltages.
  - Servo currents.
  - Servo temperature.

- Basic packaging support:
  - Version read from `bimo/__init__.py` to keep a single source of truth.

### Changed
- `initialize()`:
  - Now initializes both MCU communications and camera controls.
  - Now accepts `calibrate=False` by default so the robot can be initialized quickly on already-calibrated hardware.
  - Optionally runs the interactive servo calibration flow when `calibrate=True` before resetting to the sit pose and updating IMU offsets.

- IMU offset computation based on multiple samples:
  - `update_imu_offsets()` now averages several orientation readings with a small delay between them to reduce noise.

- Inference no longer requires gyroscope data. The model now infers angular change from orientation history, improving walking stability.

### Fixed
- Camera configuration stability over USB 2.0:
  - Forced MJPEG compression and 30 FPS to avoid bandwidth-related failures.

- IMU calibration robustness:
  - Avoids using a single noisy reading when the robot adopts the initial sitting pose and hip servos wobble slightly around the target position.

- Minor docstring and print-message inconsistencies:
  - Normalized wording, consistent capitalization, error messages, etc.

### Documentation
- Clarified supported camera resolutions and the fixed 30 FPS MJPEG behavior in docstrings and comments.

---

## [0.1.0] - 2025-12-29

### Added
- Initial public release of the Bimo Robotics Kit Python API:
  - `Bimo` class for basic robot control (MCU communication, servo commands, basic state reading).
  - `BimoRoutines` class for executing pre-programmed movements such as sitting and standing.
  - Basic integration example `api_example.py` showing control loop and model inference.

> Note: This release corresponds to the original API submit.

