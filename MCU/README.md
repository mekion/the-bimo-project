# Bimo MCU Firmware (RP2040)

This folder contains the firmware for the RP2040 microcontroller that drives Bimo's servos, reads sensors, and communicates with the Python API (BimoAPI).

## Overview

The MCU firmware handles:
- Servo control: 8 SMS-STS servos with per-joint safety limits.
- IMU sensor: quaternion orientation from the BNO08x.
- Distance sensors: 4 × VL53L0X ToF sensors via an I2C multiplexer.
- Servo feedback: position, speed, load, voltage, current, and temperature for each servo.
- System voltage reading, useful for battery-powered projects.
- RP2040 internal temperature reading.
- Power switch control to activate/deactivate servo power. Can be used as a digital safeguard.
- Serial communication: binary protocol at **921600 baud** over USB.

Most users should interact with the robot through the BimoAPI, which wraps this protocol.


## Building & Uploading

The SLS version kit (fully assembled) ships with pre-loaded firmware and calibrated servos. Check the `DIY Manual` (coming soon) for wiring and calibration instructions during the DIY build.

### Prerequisites

- Arduino IDE
- Libraries:
    - `Adafruit BNO08x`
    - `FTServo` (SMS-STS driver)
    - `VL53L0X`
    - `TCA9548A` (I2C multiplexer)


### Steps

1. Open `micro_bimo.ino` in Arduino IDE.
2. Connect the board over USB while holding the BOOT button.
3. Select **Generic RP2040** as the board (ensure RP2040 support is installed).
4. Choose the **Generic SPI/4** option in Tools > Boot Stage 2.
5. Choose the **2MB (no FS)** option in Tools > Flash Size.
6. Click **Upload**.
7. After flashing, the MCU appears as `/dev/ttyACM0` (or similar).


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

- Check USB connection: `ls /dev/ttyACM*`.
- Verify the host baud rate is **921600**.
- Send `msg == 2` (alive) and confirm a 4‑byte response.
- Ensure servos have power and are correctly connected.
- Ensure all sensors are correctly connected. MCU will fail silently with incorrect connections.
- Ensure all sensors are correctly connected. Distance sensors attempt 3 init retries on startup. Failed sensors report max range (2000 mm) silently without halting.


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

- `msg == 1` — Request state data
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

- `msg == 2` — Alive check
    - Payload: `int32(2)`
    - Response: `int32(0)` (alive/ready flag)

- `msg == 3` — Calibrate servos
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

For direct usage details, see the implementation in `micro_bimo.ino` and the Python side in `BimoAPI/bimo/bimo.py`.


### Extending Firmware

The firmware utilizes both RP2040 cores, with the main loop running fast non-blocking functions to ensure minimal delays in the communication protocol. For slower sensors, peripherals, and blocking functions, the second core is used.

- New **Core 0** peripherals: add to `loop()` following the existing pattern.
- New **Core 1** peripherals: add to `core1Entry()`, use `core1Writing` / `core0Reading` flags for any shared state writes.
- New **serial commands**: extend `processRequest()` with a new `msg` value.
- New **state fields**: extend the `StateData` struct and update `Bimo.state_format` in the Python API accordingly.


## Support

For issues or questions:
1. Review the Python `BimoAPI/bimo/bimo.py` Bimo class.
2. Open an issue on GitHub: https://github.com/mekion/the-bimo-project
