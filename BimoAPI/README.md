# BimoAPI – Python Control Library

Control the Bimo robot hardware with a simple, intuitive Python API. Run ONNX policies, execute pre-programmed routines, or build custom behaviors.

## Installation

```bash
pip install -e .
```

Installs `mekion-bimo` package with dependencies: `pyserial`, `numpy`, `onnxruntime`.

## Quick Start

### Basic Robot Control

```python
from bimo import Bimo, BimoRoutines

# Initialize robot
bimo = Bimo()
bimo.initialize()  # Sit → level IMU → stand ready

# Run pre-programmed routine
routines = BimoRoutines()
routines.perform(bimo, "stand")
routines.perform(bimo, "sit")

# Send manual servo positions (degrees)
bimo.send_positions([0, 0, 0, 0, 60, 60, 30, 30])
```

### Run ONNX Policy

```python
import onnxruntime as ort
from bimo import Bimo
import numpy as np

bimo = Bimo()
bimo.initialize()

# Load trained policy
session = ort.InferenceSession("policy.onnx")

# 50ms control loop
while True:
    imu, _, _ = bimo.request_state_data()
    # ... (build observations from IMU history) ...
    actions = session.run(None, {"input": observations})
    # ... process actions ...
    bimo.send_positions(actions)
```

See `examples/api_example.py` for a complete inference example.

## Core Classes

### `Bimo` – Robot Control

Handles MCU communication, sensor reading, and servo control.

| Method | Purpose |
|--------|---------|
| `initialize()` | Initialize from sit, calibrate IMU offset, idle ready |
| `send_positions(actions)` | Send 8 servo positions (degrees) |
| `request_state_data()` | Get IMU quaternion, distance, battery |
| `request_positions()` | Get current servo positions (degrees) |
| `calibrate_robot()` | Run full servo calibration |
| `scale_value(val, min, max)` | Scale list/value to [-1, 1] |
| `quaternion_to_euler(quat)` | Convert quaternion to Euler angles |

### `BimoRoutines` – Pre-Programmed Behaviors

Execute smooth, hand-crafted movement sequences at 50ms intervals.

| Method | Purpose |
|--------|---------|
| `perform(bimo, name)` | Execute routine: "stand", "sit", or custom |
| `add_routine(name, poses)` | Register custom routine (list of poses) |

**Example Custom Routine:**

```python
routines = BimoRoutines()
my_dance = [
    [-30, -30, 0, 0, 60, 60, 30, 30],
    [-25, -28, 0, 0, 65, 62, 32, 28],
    [-35, -32, 0, 0, 55, 58, 28, 32],
    # ... more poses ...
]
routines.add_routine("dance", my_dance)
routines.perform(bimo, "dance")
```

## Servo Configuration

Total of 8 joints, ranges in degrees:

| Index | Joint | Min | Max |
|-------|-------|-----|-----|
| 0–1 | Hips (R, L) | -90 | +90 |
| 2–3 | Shoulders (R, L) | -12, -90 | +90, +12 |
| 4–5 | Knees (R, L) | 0 | +140 |
| 6–7 | Ankles (R, L) | -93 | +93 |

## Sensor Data

### IMU (7 floats)
- Quaternion: w, x, y, z (orientation)
- Gyro: x, y, z (angular velocity, rad/s)

### Distance (4 floats)
- Readings from 4 proximity sensors (meters). Will be added in future update

### Battery (1 float)
- Power voltage or percentage. Will be added in future update

## Troubleshooting

**"Could not connect to MCU"**
- Check USB cable
- Verify RP2040 is flashed with `micro_bimo.ino` firmware
- Try: `ls /dev/ttyACM*`

**"Servos not responding"**
- Ensure robot has power
- Check servo calibration: `bimo.calibrate_robot()`
- Verify servo IDs are 0-7

## Examples

See `examples/` folder:
- **`api_example.py`** - Full RL policy inference example
- Pre-programmed sit/stand routines

## References

- [Bimo GitHub](https://github.com/mekion/the-bimo-project)
- [MCU Protocol](../MCU/README.md)
- [Isaac Lab Training](../IsaacLab/README.md)
