# Bimo MCU Firmware (RP2040)

This folder contains the firmware for the RP2040 microcontroller that drives Bimo's servos, reads sensors, and communicates with the Bimo API.

## Overview

The MCU firmware handles:
- Servo control: 8 SMS-STS servos with per-joint safety limits.
- IMU sensor readings: orientation quaternion from the BNO08x.
- Distance sensors: 4 × VL53L0X ToF sensors via an I2C multiplexer.
- Servo feedback: position, speed, load, voltage, current, and temperature for each servo.
- System readings: voltage (useful for battery-powered projects) and temperature (RP2040 internal reading).
- Power switch control to activate/deactivate servo power. Can be used as a digital safeguard.
- Serial communication: binary protocol at 921600 baud rate over USB.

Most users should interact with the robot through the Bimo API, which handles MCU communication accordingly.

## Firmware Examples

Three sketches are included. All three share the same sensor and servo initialization but differ in what drives the walking behavior and whether an external host PC is involved at all:

| Sketch | Description |
| :-- | :-- |
| `micro_bimo.ino` | The default firmware that implements the binary serial protocol, intended to be driven by a host PC or SBC running the Bimo API.|
| `micro_bimo_nn.ino` | Standalone onboard NN inference. Drops the host comms protocol entirely and runs a distilled walking policy (converted from ONNX to C) directly on the RP2040. See [Flashing a Distilled Policy](#flashing-a-distilled-policy) below for more details. |
| `micro_bimo_cpg.ino` | Standalone onboard CPG (Central Pattern Generator) gait. Same standalone structure as `micro_bimo_nn.ino`, but drives the walk from a Fourier-series CPG instead of a neural network.|

Since `micro_bimo_nn.ino` and `micro_bimo_cpg.ino` don't need a host device to talk to, they replicate inside the required functions that normally would have been handled by the Bimo API on the external device.


## Compiling & Flashing

The SLS version kit (fully assembled) ships with pre-loaded firmware and calibrated servos. Check the `DIY Manual` (coming soon) for wiring and calibration instructions during the DIY build.

### Prerequisites

- Arduino IDE
- Libraries:
    - `Adafruit BNO08x`
    - `FTServo` (SMS-STS driver)
    - `VL53L0X`
    - `TCA9548A` (I2C multiplexer)

### Steps

1. Open whichever sketch matches what you want to run, `micro_bimo.ino` (tethered), `micro_bimo_nn.ino` (standalone NN), or `micro_bimo_cpg.ino` (standalone CPG), in the Arduino IDE.

2. Connect the board over USB. If connection fails, manually mount the flash memory by holding the BOOT button when connecting.
3. Ensure the [arduino-pico](https://github.com/earlephilhower/arduino-pico) core is installed in your Arduino IDE.
4. Select **Generic RP2040** as the board.
5. Choose the **Generic SPI/4** option in Tools > Boot Stage 2.
6. Choose the **2MB (no FS)** option in Tools > Flash Size.
7. Click **Upload**.
8. After flashing, the MCU appears as `/dev/ttyACM0` (or similar) if running `micro_bimo.ino`. The two standalone sketches start walking on their own shortly after power-up and don't expose the serial protocol.

>NOTE: settings get saved in the IDE, so you don't need to input them each time you flash, only once the first time.


## Flashing a Distilled Policy

`micro_bimo_nn.ino` expects a small distilled policy converted to plain C, not a `.onnx` file directly. The RP2040 has no ONNX runtime, so the model has to be compiled into the firmware itself.

1. **Train and distill a policy:** train the walking policy and its distilled student network in Isaac Lab (see the [IsaacLab README's distillation section](../IsaacLab/README.md#train-distilled-policy) for the distillation workflow). Export the resulting student policy to `.onnx` the same way the regular PPO policy is exported (using the `play.py` script).

2. **Install `onnx2c`:** it's built from source via CMake, not pip-installable:
   ```bash
   # Protobuf is required first
   sudo apt install libprotobuf-dev protobuf-compiler   # or: brew install protobuf on macOS

   git clone https://github.com/kraiskil/onnx2c.git
   cd onnx2c
   git submodule update --init

   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make onnx2c
   ```

3. **Convert the distilled `.onnx` policy to C:**
   ```bash
   ./onnx2c policy.onnx > policy.c
   ```
   This generates a `void entry(...)` function at the end of `policy.c`, with parameter names taken from your ONNX model's input/output tensor names.

4. **Place `policy.c` in the same folder as `micro_bimo_nn.ino`:** the Arduino IDE compiles every `.c`/`.cpp` file in the sketch folder together automatically, so no separate build step or makefile edit is needed.

5. **Check the signature matches:** the `micro_bimo_nn.ino` declares the expected entry point as:
   ```cpp
   extern "C" {
       void entry(const float tensor_obs[1][11], float tensor_actions[1][8]);
   }
   ```
   This must match `policy.c`'s generated `entry()` signature exactly. In this example an 11-value observation vector goes in, and an 8-value action vector comes out, consistent with the Isaac Lab task's observation/action space.
   
   If `onnx2c` named the parameters differently (depends on your exported model's tensor names), either rename them in `policy.c` or adjust the declaration in the sketch so both sides agree.

6. Open `micro_bimo_nn.ino` in Arduino IDE with `policy.c` present in the same folder, and follow the same [Compiling & Flashing](#compiling--flashing) steps above.


## Controller Board Overview & Pinout

![Bimo Controller Board Pinout](../assets/pcb.png)

| Component | Notes |
| :-- | :-- |
| XT30 | Main power connector for power source or battery |
| Servos 1, 2| Servo connectors, one per leg|
| Cameras | Left connector: front, right connector: top |
| Distance Sensors | In order: left, front, back, right. Pinout: 3V3, GND, SDA, SCL |
| IMU | Pinout: 3V3, GND, SDA, SCL |
| Reset | Button to reboot the RP2040 |
| Boot | Hold button while plugging in the Data cable to enter flash mode and upload firmware |
| Data | Type-C USB 2.0 for communication with host PC or SBC |
| I2C Switch | Pinout of the free pins of the switch, rest are used for distance sensors |
| Buck Enable | Enables the USB-C 5V 4A output port. Wire a mechanical switch here to control SBC power. When the switch closes, the SBC boots followed by the RP2040 |
| Type-C Output | USB Type-C power output only (5V, 4A) for SBC integration |
| A0, A1, A2 | I2C Switch address selector pins. Useful when adding new I2C devices on the same bus |


> NOTE: GPIO 2, 3 are connected to IMU and GPIO 4, 5 to I2C Switch. Check addresses of new I2C devices before connecting to these pins.


## Debugging

If the MCU does not respond:

- Check USB connection: `ls /dev/ttyACM*` or equivalent in your system.

- Verify the host baud rate is **921600**.
- Send `msg == 2` (alive) and confirm a 4‑byte response.
- Ensure servos have power and are correctly connected.
- Ensure all sensors are correctly connected. MCU will fail silently with some incorrect connections.

The two standalone sketches (`micro_bimo_nn.ino`, `micro_bimo_cpg.ino`) don't expose the serial protocol, so the `msg == 2` alive check doesn't apply to them. Use `Serial` for optional logging instead if you need to debug a standalone build.


## Advanced Users

### Constants

| Constant | Value | Notes |
| :-- | --: | :-- |
| Baud Rate (host) | 921600 | `Serial.begin(921600)` |
| Baud Rate (servos) | 1,000,000 | `Serial1.begin(1000000)` |
| I2C Clock | 400 kHz | `setClock(400000)` |
| Servo Driver Pins | 0, 1 | TX/RX |
| IMU I2C Pins | 2, 3 | SDA/SCL (Wire1)|
| I2C Switch Pins | 4, 5 | SDA/SCL (Wire)|
| Power Switch Pin | 6 | Switches servo power |
| System Voltage Pin | A0 | Reads the power source voltage. Clamped to 9V-13V |
| Servo Count | 8 | IDs 0–7 |
| Servo Range | 0–4095 | Raw position units |
| Distance Sensor Range | 2000 mm | Values clamped to 2000 mm |


### Serial Protocol (high level)

All messages use the same header:

```text
[int32 msgSize][payload...]
```

#### Requests (Host → MCU)

- `msg == 1`  Request state data
    - Payload: `int32(1)`
    - Response: one packed `StateData` struct (118 bytes) containing:
        - IMU quaternion (`float[4]`, w, x, y, z).
        - Distance readings (`uint16_t[4]`, mm).
        - Power source voltage (`uint16_t`, 65535 mapped to 9V-13V).
        - RP2040 internal temperature (`float`, °C).
        - Servo feedback arrays for all 8 servos:
            - `uint16_t pos[8]`
            - `int16_t speed[8]`
            - `int16_t load[8]`
            - `uint16_t voltage[8]`
            - `int16_t current[8]`
            - `uint8_t temp[8]`

- `msg == 2` Alive check
    - Payload: `int32(2)`
    - Response: `int32(0)` (alive/ready flag)

- `msg == 3` Calibrate servos
    - Payload: `int32(3)`
    - Response: none
    - MCU runs `CalibrationOfs()` on all 8 servos.


#### Commands (Host → MCU)

- Apply servo positions
    - Payload size: 32 bytes (`8 × int32` raw servo positions).
    - MCU:
        - Copies values into `newPositions[8]`.
        - Clamps each joint using `servoMin[]` / `servoMax[]`.
        - Sends commands with `servoDriver.WritePosEx(i, pos, 3400, 0)` (max speed and torque within safety limits).

For direct usage details, see the implementation in `micro_bimo.ino` and the Python side in `BimoAPI/bimo/bimo.py`. The two standalone sketches don't implement this protocol at all. Everything they need is computed and applied locally.


### Extending Firmware

The firmware utilizes both RP2040 cores, with the main loop running fast non-blocking functions to ensure minimal delays in the communication protocol. For slower sensors, peripherals, and blocking functions, the second core is used. This pattern applies to all three sketches, though only `micro_bimo.ino` has an actual communication protocol to keep responsive.

- New **Core 0** peripherals: add to `loop()` following the existing pattern.

- New **Core 1** peripherals: add to `core1Entry()`, use `core1Writing` / `core0Reading` flags for any shared state writes.
- New **serial commands** (`micro_bimo.ino` only): extend `processRequest()` with a new `msg` value.
- New **state fields**: extend the `StateData` struct and update `Bimo.state_format` in the Python API accordingly.

>NOTE: the standalone sketches use a locally-scoped `SensorSnapshot` struct instead of `StateData`, since they don't serialize it over the wire. Adding a field there doesn't require touching the Python API.


## Support

For issues or questions:
1. Review the `Bimo` class in the Bimo API `BimoAPI/bimo/bimo.py` and `micro_bimo.ino` sketch.
2. Open an issue on GitHub: https://github.com/mekion/the-bimo-project/issues.

---