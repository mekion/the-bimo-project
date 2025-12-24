#include <SCServo.h>
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BNO08x.h"

// Host ↔ MCU serial protocol (921600 baud)
//
// Message format from host:
//   [int32 msgSize][payload...]
//
// If msgSize == 4:
//   payload = int32 command:
//     1 = request state   -> MCU replies with 48 bytes: IMU[7 floats], dist[4 floats], battery[1 float]
//     2 = alive check     -> MCU replies with int32(1)
//     3 = request positions -> MCU replies with 8×int32 (current servo positions)
//     4 = calibrate servos
//
// If msgSize == 32:
//   payload = 8×int32 -> absolute servo positions, applied immediately.

// I2C DEFINITION
#define IMU_WIRE Wire1

// IMU
Adafruit_BNO08x  imu;
sh2_SensorValue_t imuValue;

float imuData[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Servos
SMS_STS servoDriver;

int currPositions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int newAction[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Distance Sensors
float distData[4] = {120.5, 100.47, 167.233, 20.44};  // Dummy values

// Battery
float battery = 100.0;  // Dummy value


void setup()
{
  Serial.begin(921600);
  delay(300);

  // Servo Driver
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(1000000);
  delay(300);

  servoDriver.pSerial = &Serial1;

  // Wire1 & IMU Init
  IMU_WIRE.setSDA(10);
  IMU_WIRE.setSCL(11);
  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  delay(300);

  imu.begin_I2C(0x4B, &IMU_WIRE);
  imu.enableReport(SH2_GYRO_INTEGRATED_RV);
  // imu.enableReport(SH2_ACCELEROMETER);  Too noisy for realtime control
}

void loop()
{
  updateIMU();
  checkComms();
}


void checkComms()
{
  if (Serial.available() >= 4)
  {
    // Reads first 4 bytes = packet size
    int msgSize = 0;
    uint8_t controlBytes[4] = {0};
    
    Serial.readBytes(controlBytes, 4);
    memcpy(&msgSize, controlBytes, 4);

    // Reads full message
    uint8_t inBuffer[msgSize] = {0};
    Serial.readBytes(inBuffer, msgSize);

    // Manages message
    if (msgSize == 4)
    {
      // Single byte SoC message -> process request
      int socMessage = 0;
      memcpy(&socMessage, inBuffer, 4);
      processRequest(socMessage);
    }

    else if (msgSize == 32)
    {
      // Applies new actions immediately
      memcpy(newAction, inBuffer, 32);
      applyPositions(newAction);
    }
  }
}


void processRequest(int msg)
{
  if (msg == 1)
  {
    // SoC asking for state data
    sendStateData();
  }

  else if (msg == 2)
  {
    // SoC asking for alive status
    uint8_t outBuffer[4] = {0};
    
    int response = 1;
    
    memcpy(outBuffer, &response, 4);
    Serial.write(outBuffer, 4);
  }

  else if (msg == 3)
  {
    // SoC asking for current positions
    uint8_t outBuffer[32] = {0};
    
    readCurrentPositions();
    memcpy(outBuffer, currPositions, 32);

    Serial.write(outBuffer, 32);
  }
  
  else if(msg == 4)
  {
    // SoC asking to calibrate
    for (int i = 0; i < 8; i++)
    {
      servoDriver.CalibrationOfs(i);
    }
  }
}


void sendStateData()
{
  // Sends state data [IMU, x4 Distance Sensors, Battery]
  uint8_t outBuffer[48];

  // Append leading byte (message size) and state data
  memcpy(outBuffer, imuData, 28);
  memcpy(outBuffer + 28, distData, 16);
  memcpy(outBuffer + 44, &battery, 4);

  // Send state data
  Serial.write(outBuffer, 48);
}


void updateIMU()
{
  // Updates IMU sensor data array
  if (imu.getSensorEvent(&imuValue))
  {
    if (imuValue.sensorId == SH2_GYRO_INTEGRATED_RV)
    {
      // Quaternion Geomagnetic Orientation Data
      imuData[0] = imuValue.un.gyroIntegratedRV.real;
      imuData[1] = imuValue.un.gyroIntegratedRV.i;
      imuData[2] = imuValue.un.gyroIntegratedRV.j;
      imuData[3] = imuValue.un.gyroIntegratedRV.k;
      
      // Gyro
      imuData[4] = imuValue.un.gyroIntegratedRV.angVelX;
      imuData[5] = - imuValue.un.gyroIntegratedRV.angVelY;
      imuData[6] = imuValue.un.gyroIntegratedRV.angVelZ;
    }
  }
}


void readCurrentPositions()
{
  // Gets current servo positions
  for(int i = 0; i < 8; i++)
  {
    currPositions[i] = servoDriver.ReadPos(i);
  }
}


void applyPositions(int* positions)
{
  // Applies new positions
  for(int i = 0; i < 8; i++)
  {
    servoDriver.WritePosEx(i, positions[i], 4095, 254);
  }
}


void updateServoId(int original, int newId)
{
  // Sets new ID to servo
  servoDriver.unLockEprom(original);
  servoDriver.writeByte(original, SMS_STS_ID, newId);
  servoDriver.LockEprom(newId);
}
