

# BimoAPI – Python Control Library

Control the Bimo robot with a simple Python API. Run ONNX policies, CPG models, execute pre-programmed routines, or build custom behaviors.

## Installation

From the `BimoAPI` folder:

```bash
pip install -e .
```

This installs the `mekion-bimo` package and dependencies (`pyserial`, `numpy`, `onnxruntime`, `opencv-python`, `pynput`).

## Quick Start Guide

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


### Full Control Examples

Three ready-to-run scripts are included, each a complete standalone control loop built on the Bimo API:

| Script | Description |
| :-- | :-- |
| `cpg_walk.py` | CPG-driven walking gait with startup ramp-up and automatic heading-hold turn correction. No input required. Walks straight once standing. |
| `cpg_walk_keyboard.py` | Same CPG gait as above, but steerable in real time: hold `Q`/`E` to turn left/right, release to resume walking straight. |
| `nn_walk.py` | ONNX policy inference loop. Loads a trained `policy.onnx`, builds observations from orientation and last actions each step, and executes the model's actions output. |

> Both CPG examples turn by biasing the two hip joints in opposite directions on top of their straight-walk offset `CENTER_VALUE`, while `MAX_TURN` caps how aggressive the bias can get. `CENTER_VALUE` is a per-robot calibration constant and may need adjusting if your build drifts while walking straight.

> The `cpg_walk.py` example derives that bias automatically, proportionally to the heading deviation, to hold a straight line.

> The `cpg_walk_keyboard.py` example overrides it directly from `Q`/`E` key state. 

Run any of the examples directly once the robot is wired up.  The  `policy.onnx` file (for the `nn_walk.py` example) needs to be in the working directory for correct execution.

```bash
# Launching examples
python3 cpg_walk.py
python3 cpg_walk_keyboard.py
python3 api_example.py
```

Read through whichever one matches your use case for the exact `lock_heading()` placement, ramp-up handling, camera control and observation construction. See the **API Reference** below for more information.


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
| `get_routine(name)` | Returns a routine by name (list of actions). |

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

### `BimoCPG`: central pattern generator

Fourier-series gait approximation with per-joint amplitude gain correction. Runs independently of `Bimo`.  Instantiate it separately and feed its output into `bimo.send_positions()` yourself, as shown in `cpg_walk.py` and `cpg_walk_keyboard.py` examples.

```python
# CPG initalization
from bimo import BimoCPG
cpg = BimoCPG(step_freq_hz=1.5)
```

**Constructor**

| Parameter | Description |
| :-- | :-- |
| `step_freq_hz` (default `1.5`) | Cycles per second of the gait (`1 / step_period`). |
| `phase_offset` (default `0.0`) | Global phase shift, in radians. |
| `amp_gain` (default `None`) | Optional `{joint_idx: gain}` dict overriding the built-in per-joint amplitude gains, e.g. `amp_gain={4: 2.5, 5: 2.5}` to push the knees further if they undershoot. |

**Attributes**

- `omega`: angular frequency in rad/s (`2π × step_freq_hz`). Can be reassigned at runtime to change gait speed on the fly. This is how both CPG examples ramp the gait in from a standstill.

- `phi`: current internal phase, in radians (`0` to `2π`).
- `amp_gain`: the active per-joint amplitude gain dict.

**Methods**

| Method | Description |
| :-- | :-- |
| `step(dt)` | Advances the internal phase clock by `dt` seconds. |
| `joint_angle(j)` | Returns the next position (degrees) for joint `j` (0–7) at the current phase. |
| `angles()` | Returns the next positions (degrees) for all 8 joints as a list, in the same joint order as `Bimo.send_positions()`. |

> NOTE: `BimoCPG` only computes joint angles. It never talks to the MCU. Call `angles()` to get positions, then pass them to `bimo.send_positions()` yourself, and call `step(dt)` once per loop iteration to advance the gait.


## Cameras

Cameras run over a separate USB/V4L2 path from the MCU serial connection, so `capture_image()` never competes with `send_positions()`/`request_state_data()` for the same hardware.  You can safely call both in the same control loop. 

However, `capture_image()` itself is a **blocking** call (it waits on the camera driver to hand back a frame), so if you need to stream images continuously alongside a tight control loop, run the capture calls on their own thread rather than inline in the loop.  Otherwise every frame grab stalls that iteration's timing, even though nothing about the MCU communication requires it to.

- Two USB cameras are expected (front and top).

- Supported resolutions (MJPEG @ 30 FPS): `(1280, 720)`, `(848, 480)`, `(800, 600)`, `(640, 480)`, `(640, 360)`, `(352, 288)`, `(320, 240)`, `(160, 120)`.
- `Bimo.initialize()` automatically detects and configures them.

```python
# Example usage
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
    - Ensure the two cameras are connected.
    - Check `/dev/video*` and supported resolutions.

---
