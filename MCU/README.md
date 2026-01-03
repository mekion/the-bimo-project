# Bimo MCU Firmware (RP2040)

This folder contains the firmware for the RP2040 microcontroller that drives Bimo's servos, reads sensors, and communicates with the Python API.

## Overview

The MCU firmware handles:
- **Servo control** - Commands 8 servo motors
- **IMU sensor** - Reads quaternion orientation and angular velocity from IMU
- **Distance sensors** - Provides distance readings from 4 proximity sensors (Future Update)
- **Battery monitoring** - Reports power voltage (Future Update)
- **Serial communication** - Exchanges data with the Python control API (BimoAPI)

All communication happens over a binary serial protocol at **921600 baud**.

## Serial Protocol

Communication is **message-based** with a fixed header format:

```
[int32 msgSize][payload...]
```

### Request Messages (Host → MCU)

#### 1. Request State Data (msgSize = 4)
```
Command: int32(1)
Response: 48 bytes
  - Bytes 0–27:   IMU data (7 floats)
  - Bytes 28–43:  Distance data (4 floats)
  - Bytes 44–47:  Battery (1 float)
```

**Response Structure:**
| Field | Bytes | Type | Content |
|-------|-------|------|---------|
| Quaternion | 0–15 | 4 floats (w, x, y, z) | Orientation Quaternion|
| Angular Velocity | 16–27 | 3 floats (x, y, z) | Gyro data in rad/s |
| Distance 0–3 | 28–43 | 4 floats | ToF sensor readings (m) |
| Battery | 44–47 | 1 float | Voltage (V) or percentage |

#### 2. Alive Check (msgSize = 4)
```
Command: int32(2)
Response: int32(1)  // Always responds with 1 if alive
```

#### 3. Request Servo Positions (msgSize = 4)
```
Command: int32(3)
Response: 32 bytes (8 × int32)
  - Current absolute position of each servo in raw units (0–4095)
```

#### 4. Calibrate Servos (msgSize = 4)
```
Command: int32(4)
Response: None
  - MCU calls CalibrationOfs() for each of the 8 servos
  - Blocks until calibration completes
```

### Command Messages (Host → MCU)

#### Apply Servo Positions (msgSize = 32)
```
Payload: 8 × int32 (32 bytes total)
  - Bytes 0–3:   Servo 0 absolute position (0–4095)
  - Bytes 4–7:   Servo 1 absolute position
  - ...
  - Bytes 28–31: Servo 7 absolute position

Response: None (positions applied immediately)
Speed: 4095 (max), Torque: 254 (max)
```

## Python Integration Example

```python
import struct
import serial

ser = serial.Serial("/dev/ttyACM0", 921600, timeout=0.2)

# Request state
msg = struct.pack("i", 1)  # Command 1
ser.write(struct.pack("i", len(msg)))  # Header
ser.write(msg)

# Read response
response = ser.read(48)
imu = struct.unpack("7f", response[:28])
dist = struct.unpack("4f", response[28:44])
batt = struct.unpack("f", response[44:48])

print(f"Quat: {imu[:4]}, Gyro: {imu[4:7]}")
print(f"Distance: {dist}, Battery: {batt[0]}")

# Send servo positions
positions = [2048] * 8  # Center all servos
payload = struct.pack("8i", *positions)
ser.write(struct.pack("i", len(payload)))  # Header
ser.write(payload)
```

The BimoAPI package handles all of this automatically. See `BimoAPI/bimo/bimo.py` for the full implementation.

## Building & Uploading

### Prerequisites

- Arduino IDE
- Libraries:
  - `Adafruit BNO08x`
  - `FTServo`

### Steps

1. Open `micro_bimo.c` in Arduino IDE
2. Select **RP2040 Generic** as board (ensure RP2040 libraries are installed)
3. Connect USB serial port
4. Click **Upload**
5. After flashing, the MCU appears as `/dev/ttyACM0` (or `/dev/ttyACM1`, etc.)


## Debugging

If the MCU doesn't respond:

1. **Check USB connection**: `ls /dev/ttyACM*`
2. **Check baud rate**: Must be exactly 921600
3. **Test alive command**: Send `int32(2)` and expect `int32(1)` back
5. **Check Servos**: verify servos have power.

## Message Flow Example

Typical 50ms control loop:

```
[0ms]   Host sends "state" (cmd 1)
[1ms]   MCU responds with 48 bytes
[5ms]   Host sends 8 servo positions (~5ms NN inference or ~1ms if routine)
[50ms]  Loop repeats
```

This ensures:
- Real-time servo command feedback
- Current joint feedback for policy input
- Non-blocking IMU reads
- Fixed 50ms control frequency

## Constants

| Constant | Value | Notes |
|----------|-------|-------|
| Baud rate (host) | 921600 | Serial.begin(921600) |
| Baud rate (servos) | 1,000,000 | Serial1.begin(1000000) |
| I2C clock (IMU) | 400 kHz | IMU_WIRE.setClock(400000) |
| IMU address | 0x4B | Adafruit BNO08x default |
| Servo count | 8 | Numbering: 0–7 |
| Servo range | 0–4095 | Raw position units |
| Max servo speed | 4095 | WritePosEx param 2 |
| Max servo torque | 254 | WritePosEx param 3 |

## Future Improvements

- Implement battery voltage ADC reading
- Add distance sensor reading from multiple sensors (threading)
- Add on-MCU servo safety limits
- Add toggleable servo feedback (current, temperature, speed...)

**Note:** these software functions will be implemented once all required tests are passed on the main PCB.

## References

- **RP2040 Datasheet**: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- **BNO08x Adafruit Library**: https://github.com/adafruit/Adafruit_BNO08x
- **SMS-STS Servo Protocol**: Provided by servo manufacturer

## Support

For issues or questions:
1. Review the Python `BimoAPI/bimo/bimo.py` Bimo class
3. Open an issue on GitHub: https://github.com/mekion/the-bimo-project
