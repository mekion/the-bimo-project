# BimoAPI – Python Control Library

Control the Bimo robot with a simple Python API. Run ONNX policies, execute pre-programmed routines, or build custom behaviors.

## Installation

From the `BimoAPI` folder:

```bash
pip install -e .
```

This installs the `mekion-bimo` package and dependencies (`pyserial`, `numpy`, `onnxruntime`, `opencv-python`).

## Quick Start

### Basic Usage

```python
from bimo import Bimo

bimo = Bimo()
bimo.initialize()  # Connect MCU + cameras, go to sit, calibrate IMU

bimo.perform("stand")  # Stand up
bimo.perform("sit")    # Sit down

# Send manual servo positions (degrees)
bimo.send_positions([-30, -30, 0, 0, 60, 60, 30, 30])
```
> WARNING: Always double-check manual positions! The above example sends a standing pose as an example AFTER sitting down, which would make the robot launch itself backwards!

### ONNX Policy Loop (`api_example.py`)

```python
from time import sleep, time
import onnxruntime as ort
import numpy as np
from bimo import Bimo

bimo = Bimo()
bimo.initialize()
bimo.perform("stand")
bimo.lock_heading()  # Required for walking model

session = ort.InferenceSession(
    "policy.onnx",
    providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
)

period = 0.05
t1 = time()

while True:
    state = bimo.request_state_data()  # Full state dictionary
    # ... build observations from orientation + action history ...
    actions = session.run(None, {session.get_inputs()[0].name: obs.reshape(1, -1)})
    # ... post-process actions ...
    bimo.send_positions(new_actions)  # Degrees

    t1 += period
    sleep_dt = max(0, t1 - time())
    sleep(sleep_dt)
```
> NOTE: `lock_heading()` must be called after standing. The walking model observes yaw as deviation from this reference point. Without it, heading correction will fail.


## API Reference

### `Bimo`: robot control

Main entry point to talk to the robot.

**Main Attributes**

- `sit_pose`: default sitting pose joint configuration (degrees).
- `stand_pose`: default standing pose joint configuration (degrees).
- `servo_min`, `servo_max`: per-joint limits (degrees).


**Main Methods**

| Method | Description |
| :-- | :-- |
| `initialize(calibrate=False, baudrate=921600, timeout=0.2, camera_resolution=(1280, 720))` | Connect to MCU and cameras, optionally run servo calibration, move to sit pose, calibrate IMU. |
| `request_state_data()` | Return a dict with orientation, distances, and servo feedback. |
| `send_positions(actions)` | Send 8 joint positions in degrees to the MCU. |
| `perform(name)` | Execute a named routine directly. Built-in: "stand", "sit". |
| `lock_heading()` | Set current yaw as heading reference. Required before running walking models. |
| `unlock_heading()` | Clear heading reference, restoring raw IMU yaw. |
| `clip_actions(array)` | Clamp a degree list in-place to per-joint servo limits. |
| `capture_image(camera="front")` | Capture a single frame from `"front"` or `"top"` camera as a BGR `numpy` array. |
| `scale_value(values, mins, maxs)` | Scale a value or list from `[min, max]` to `[-1, 1]` (used for observations). |
| `available()` | Check if MCU is alive and responding. |
| `port()` | Return the serial port used to connect to the MCU. |

**State Data Structure**

```python
state = bimo.request_state_data()

state["orient"]         # [roll, pitch, yaw] in radians (Euler)
state["distances"]      # [front, back, right, left] in meters
state["power"]          # Current system voltage reading (V)
state["rp_temp"]        # RP2040 internal temperature (°C)
state["servo_pos"]      # 8 joint positions in degrees
state["servo_speed"]    # 8 joint speeds in rad/s
state["servo_load"]     # 8 joint loads in Nm (approximate)
state["servo_voltage"]  # 8 joint voltages in V
state["servo_current"]  # 8 joint currents in A
state["servo_temp"]     # 8 joint temperatures in °C
```


### `BimoRoutines`: pre-programmed motions

Time-based sequences running at 50 ms per step.

**Built-in Routines**

Built-in routines ("stand", "sit") are available directly via bimo.perform(). For custom routines, instantiate BimoRoutines separately and register them with add_routine() before calling bimo.perform().

- `"stand"` Transition from sit to stable stand.
- `"sit"` Transition from stand to stable sit.

**Methods**

| Method | Description |
| :-- | :-- |
| `add_routine(name, poses)` | Register a custom routine as a list of 8‑element pose lists (degrees). |

> NOTE: custom routines are added via `bimo.routines.add_routine(name, poses)` and executed with `bimo.perform(name)`.


**Example Custom Routine**

```python
from bimo import Bimo

bimo = Bimo()
bimo.initialize()

wobble_head = [
    bimo.stand_pose,
    [-32, -32, 0, 0, 60, 60, 30, 30],
    [-31, -31, 0, 0, 60, 60, 30, 30],
    [-30, -30, 0, 0, 60, 60, 30, 30],
    [-29, -29, 0, 0, 60, 60, 30, 30],
    [-28, -28, 0, 0, 60, 60, 30, 30],
    [-29, -29, 0, 0, 60, 60, 30, 30],
    [-30, -30, 0, 0, 60, 60, 30, 30],
    [-31, -31, 0, 0, 60, 60, 30, 30],
    [-32, -32, 0, 0, 60, 60, 30, 30],
] * 5

bimo.routines.add_routine("wobble", wobble_head)
bimo.perform("wobble")
```


## Cameras

- Two USB cameras are expected (front and top).
- Supported resolutions (MJPEG @ 30 FPS): `(1280, 720)`, `(848, 480)`, `(800, 600)`, `(640, 480)`, `(640, 360)`, `(352, 288)`, `(320, 240)`, `(160, 120)`.
- `Bimo.initialize()` automatically detects and configures them.

```python
front = bimo.capture_image("front")
top = bimo.capture_image("top")
```


## Troubleshooting

- **Cannot connect to MCU**:
    - Check USB cable and that the RP2040 is flashed with the latest `micro_bimo.ino`.
    - Check `/dev/ttyACM*` and permissions.
- **Servos not moving**:
    - Verify robot power and servo wiring.
    - Run `bimo.initialize(calibrate=True)` once after finishing the DIY build.
- **Cameras not found**:
    - Ensure two UVC cameras are connected.
    - Check `/dev/video*` and supported resolutions.

For a full working example, see `api_example.py`.
