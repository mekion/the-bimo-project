# Bimo MCU Firmware (RP2040)

This folder contains the firmware for the RP2040 microcontroller that drives Bimo's servos, reads sensors, and communicates with the Python API (BimoAPI).

The firmware version is `0.9.5-Beta` and targets the RP2040 MCU.

## Overview

The MCU firmware handles:
- Servo control: 8 SMS-STS servos with per-joint safety limits.
- IMU sensor: quaternion orientation from the BNO08x.
- Distance sensors: 4 × VL53L0X ToF sensors via an I2C multiplexer.
- Servo feedback: position, speed, load, voltage, current, and temperature for each servo.
- Serial communication: binary protocol at **921600 baud** over USB.

Most users should interact with the robot through the Bimo Python API, which wraps this protocol.

## Serial Protocol (high level)

All messages use the same header:

```text
[int32 msgSize][payload...]
```

### Requests (Host → MCU)

- `msg == 1` — Request state data
    - Payload: `int32(1)`
    - Response: one packed `StateData` struct (112 bytes) containing:
        - IMU quaternion (`float[^4]`, w, x, y, z).
        - Distance readings (`uint16_t[^4]`, mm).
        - Servo feedback arrays for all 8 servos:
            - `uint16_t pos[^8]`
            - `int16_t speed[^8]`
            - `int16_t load[^8]`
            - `uint16_t voltage[^8]`
            - `int16_t current[^8]`
            - `uint8_t temp[^8]`
- `msg == 2` — Alive check
    - Payload: `int32(2)`
    - Response: `int32(0)` (alive/ready flag)
- `msg == 3` — Calibrate servos
    - Payload: `int32(3)`
    - Response: none
    - MCU runs `CalibrationOfs()` on all 8 servos.


### Commands (Host → MCU)

- Apply servo positions
    - Payload size: 32 bytes (`8 × int32` raw servo positions).
    - MCU:
        - Copies values into `newPositions[^8]`.
        - Clamps each joint using `servoMin[]` / `servoMax[]`.
        - Sends commands with `servoDriver.WritePosEx(i, pos, 3400, 254)` (max speed and torque within safety limits).

For direct usage details, see the implementation in `micro_bimo.ino` and the Python side in `BimoAPI/bimo/bimo.py`.

## Building \& Uploading

### Prerequisites

- Arduino IDE
- Libraries:
    - `Adafruit BNO08x`
    - `FTServo` (SMS-STS driver)
    - `VL53L0X`
    - `TCA9548A` (I2C multiplexer)


### Steps

1. Open `micro_bimo.ino` in Arduino IDE.
2. Select **RP2040 Generic** as the board (ensure RP2040 support is installed).
3. Connect the board over USB.
4. Click **Upload**.
5. After flashing, the MCU appears as `/dev/ttyACM0` (or similar).

## Debugging

If the MCU does not respond:

- Check USB connection: `ls /dev/ttyACM*`.
- Verify the host baud rate is **921600**.
- Send `msg == 2` (alive) and confirm a 4‑byte response.
- Ensure servos have power and are correctly connected.


## Constants

| Constant | Value | Notes |
| :-- | --: | :-- |
| Baud rate (host) | 921600 | `Serial.begin(921600)` |
| Baud rate (servos) | 1,000,000 | `Serial1.begin(1000000)` |
| I2C clock | 400 kHz | `setClock(400000)` |
| IMU address | 0x4B | BNO08x |
| Servo count | 8 | IDs 0–7 |
| Servo range | 0–4095 | Raw position units |
| Distance max range | 2000 mm | Values clamped to 2000 mm |


## Support

For issues or questions:
1. Review the Python `BimoAPI/bimo/bimo.py` Bimo class
2. Open an issue on GitHub: https://github.com/mekion/the-bimo-project
