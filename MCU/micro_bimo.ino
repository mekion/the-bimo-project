/*
 * MCU Firmware for Bimo Robotics Kit
 * Copyright (c) 2025-2026, Mekion
 * SPDX-License-Identifier: Apache-2.0
 *
 * Version: 0.9.5-Beta
 * Target: RP2040
 */

#include <Arduino.h>
#include <pico/multicore.h>
#include <Wire.h>
#include <SCServo.h>
#include <Adafruit_BNO08x.h>
#include <VL53L0X.h>
#include <TCA9548A.h>

// I2C Definition
#define MUX_WIRE Wire
#define IMU_WIRE Wire1

// Handshake flags
volatile bool core0Reading = false;
volatile bool core1Writing = false;

// IMU
Adafruit_BNO08x  imu;
sh2_SensorValue_t imuValue;

// I2C Mux
TCA9548A mux;

// Distance Sensors
VL53L0X distSensors[4];
uint16_t distBuffer[4] = {0};  // Buffer for core 1

// Servos
SMS_STS servoDriver;
int newPositions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int servoMin[8] = {485, 1563, 1911, 1024, 456, 2041, 984, 984};
const int servoMax[8] = {2533, 3611, 3072, 2185, 2055, 3640, 3112, 3112};

// State Data, total 112B
struct StateData {
  // IMU 16B
  float imu[4];
  
  // Distance 8B
  uint16_t dist[4];
  
  // Servo feedback 88B
  uint16_t pos[8];  // 16B
  int16_t speed[8];  // 16B
  int16_t load[8];  // 16B
  uint16_t voltage[8];  // 16B
  int16_t current[8];  // 16B
  uint8_t temp[8];  // 8B
} state;


void setup()
{
  Serial.begin(921600);
  while (!Serial && millis() < 3000);

  // Servo Driver
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(1000000);
  while (!Serial1 && millis() < 3000);

  servoDriver.pSerial = &Serial1;

  // Wire, Mux, Distance Sensors
  MUX_WIRE.setSDA(4);
  MUX_WIRE.setSCL(5);
  MUX_WIRE.begin();
  MUX_WIRE.setClock(400000);
  delay(100);

  mux.begin(MUX_WIRE);
  mux.closeAll();

  for (int i = 0; i < 4; i++)
  {
    mux.openChannel(i);
    distSensors[i].init();
    distSensors[i].setTimeout(500);
    distSensors[i].setMeasurementTimingBudget(33000);
    distSensors[i].startContinuous(33);  // 33ms interval
    mux.closeChannel(i);
  }

  // Wire1 & IMU
  IMU_WIRE.setSDA(10);
  IMU_WIRE.setSCL(11);
  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  delay(100);

  imu.begin_I2C(0x4B, &IMU_WIRE);
  imu.enableReport(SH2_GYRO_INTEGRATED_RV);

  // Launch core 1
  multicore_launch_core1(core1Entry);
}


void core1Entry()
{
  while(true)
  {
    // Measures new distance sensor readings
    for (int i = 0; i < 4; i++)
    {
      mux.openChannel(i);
      uint16_t d = distSensors[i].readRangeContinuousMillimeters();

      if (distSensors[i].timeoutOccurred() || d >= 2000)
      {
        distBuffer[i] = 2000;  // Max range (mm)
      }
      else
      {
        distBuffer[i] = d;
      }
      mux.closeChannel(i);
    }

    // Writes new distances to state struct
    core1Writing = true;
    while(core0Reading) delay(0);

    for (int i = 0; i < 4; i++)
    {
      state.dist[i] = distBuffer[i];
    }

    core1Writing = false;
  }
}


void loop()
{
  updateIMU();
  updateServoFeedback();
  checkComms();
}


void checkComms()
{
  // Handles incoming messages
  if (Serial.available() >= 4)
  {
    // Reads first 4 bytes = message size
    uint8_t controlBytes[4] = {0};
    Serial.readBytes(controlBytes, 4);

    int msgSize = 0;
    memcpy(&msgSize, controlBytes, 4);

    // Reads full message
    uint8_t inBuffer[msgSize] = {0};
    Serial.readBytes(inBuffer, msgSize);

    // Manages message
    if (msgSize == 4)
    {
      // Single int message -> process request
      int request = 0;
      memcpy(&request, inBuffer, 4);
      processRequest(request);
    }

    else if (msgSize == 32)
    {
      // Applies new actions immediately
      memcpy(newPositions, inBuffer, 32);
      applyNewPositions();
    }
  }
}


void processRequest(int msg)
{
  if (msg == 1)
  {
    // Asking for state data
    core0Reading = true;
    while(core1Writing) delay(0);
    Serial.write((uint8_t*)&state, sizeof(state));
    core0Reading = false;
  }

  else if (msg == 2)
  {
    // Asking for alive status
    uint8_t outBuffer[4] = {0};
    int response = 0;
    
    memcpy(outBuffer, &response, 4);
    Serial.write(outBuffer, 4);
  }

  else if (msg == 3)
  {
    // Asking to calibrate
    for (int i = 0; i < 8; i++)
    {
      servoDriver.CalibrationOfs(i);
    }
  }
}


void updateIMU()
{
  // Updates IMU data readings
  if (imu.getSensorEvent(&imuValue))
  {
    if (imuValue.sensorId == SH2_GYRO_INTEGRATED_RV)
    {
      // Quaternion Geomagnetic Orientation Data
      state.imu[0] = imuValue.un.gyroIntegratedRV.real;
      state.imu[1] = imuValue.un.gyroIntegratedRV.i;
      state.imu[2] = imuValue.un.gyroIntegratedRV.j;
      state.imu[3] = imuValue.un.gyroIntegratedRV.k;
      
      // Gyro: not needed for baseline models
      // state.imu[4] = imuValue.un.gyroIntegratedRV.angVelX;
      // state.imu[5] = - imuValue.un.gyroIntegratedRV.angVelY;
      // state.imu[6] = imuValue.un.gyroIntegratedRV.angVelZ;
    }
  }
}


void updateServoFeedback()
{
  // Updates current servo feedback data
  for (int i = 0; i < 8; i++)
  {
    if (servoDriver.FeedBack(i) != -1)
    {
      state.pos[i] = servoDriver.ReadPos(-1);
      state.speed[i] = servoDriver.ReadSpeed(-1);
      state.load[i] = servoDriver.ReadLoad(-1);
      state.voltage[i] = servoDriver.ReadVoltage(-1);
      state.current[i] = servoDriver.ReadCurrent(-1);
      state.temp[i] = servoDriver.ReadTemper(-1);
    }
  }
}


void applyNewPositions()
{
  // Applies new positions
  for (int i = 0; i < 8; i++)
  {
    int pos = newPositions[i];

    // Ensures actuator limits
    if (pos <= servoMin[i])
    {
      pos = servoMin[i];
    }
    else if (pos >= servoMax[i])
    {
      pos = servoMax[i];
    }
    
    // Applies action
    servoDriver.WritePosEx(i, pos, 3400, 254);
  }
}
