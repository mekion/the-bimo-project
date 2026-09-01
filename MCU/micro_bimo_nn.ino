/*
 * Copyright (c) 2026, Mekion
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * MCU Firmware for Bimo Robotics Kit | Standalone NN Deployment
 *
 *
 * > Onboard NN inference for a distilled walking policy.
 * 
 * > Preserves core sensor/servo init from micro_bimo.ino, drops
 * the comms protocol.
 *
 * > Observation vector uses the latest orientation + latest
 * commanded action.
 *
 *
 * Version: 1.0.0
 * Target: RP2040
 */

#include <Arduino.h>
#include <pico/multicore.h>
#include <Wire.h>
#include <SCServo.h>
#include <Adafruit_BNO08x.h>
#include <VL53L0X.h>
#include <TCA9548A.h>

// I2C definitions
#define MUX_WIRE Wire
#define IMU_WIRE Wire1

// Inference loop definitions
#define PERIOD 50  // ms
#define IMU_OFFSET_SAMPLES 10

// Distilled policy entry point
extern "C" {
  void entry(const float tensor_obs[1][11], float tensor_actions[1][8]);
}

// NN Model I/O buffers
float infObs[1][11] = {0.0};  // Observation buffer: [orient(3), last_action(8)]
float infAct[1][8] = {0.0};  // Raw actions buffer (delta, degrees/step)
float lastPositions[8] = {-30, -30, 0, 0, 60, 60, 30, 30};  // Last commanded positions (same as stand pose on start)

unsigned long timer = 0; // Timekeeping for inference loop

// Handshake flags
volatile bool core0Reading = false;
volatile bool core1Writing = false;

// IMU
Adafruit_BNO08x imu;
sh2_SensorValue_t imuValue;

float calX = 0.0f; // IMU calibration offset X
float calY = 0.0f; // IMU calibration offset Y
float baseHeading = 0.0;  // For heading lock

// I2C Mux
TCA9548A mux;

// Distance sensors
VL53L0X distSensors[4];
uint16_t distBuffer[4] = {0};

// Servos
SMS_STS servoDriver;
const int servoMin[8] = {485, 1563, 1911, 1024, 456, 2041, 984, 984};
const int servoMax[8] = {2533, 3611, 3072, 2185, 2055, 3640, 3112, 3112};

float servoDegMax[8] = {90, 90, 90, 12, 140, 140, 93, 93};
float servoDegMin[8] = {-90, -90, -12, -90, 0, 0, -93, -93};

int centers[8] = {1508, 2588, 2048, 2048, 456, 3640, 2048, 2048};
float standPose[8] = {-30, -30, 0, 0, 60, 60, 30, 30};
float sitPose[8] = {-47.5, -47.5, 0, 0, 140, 140, 93, 93};

// State Data
struct SensorSnapshot {
  float imu[4] = {0};
  uint16_t dist[4] = {0};
  uint16_t power = 0;
  uint16_t pos[8] = {0};
  int16_t speed[8] = {0};
  int16_t load[8] = {0};
  uint16_t voltage[8] = {0};
  int16_t current[8] = {0};
  uint8_t temp[8] = {0};
} state;


void setup() {
  // Servo power switch pin
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  // Power source read pin
  pinMode(A0, INPUT);
  analogReadResolution(12);

  // Main serial (optional for logging)
  Serial.begin(921600);
  while (!Serial && millis() < 3000);

  // Servo driver
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

  for (int i = 0; i < 4; i++) {
    mux.openChannel(i);
    delay(25);

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
      ok = distSensors[i].init();
      if (!ok) {
        delay(50);
      }
    }

    if (ok) {
      distSensors[i].setTimeout(200);
      distSensors[i].setMeasurementTimingBudget(33000);
      distSensors[i].startContinuous(33);
    }

    mux.closeChannel(i);
    delay(25);
  }

  // Wire1 & IMU
  IMU_WIRE.setSDA(2);
  IMU_WIRE.setSCL(3);
  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  delay(100);

  imu.begin_I2C(0x4B, &IMU_WIRE);
  imu.enableReport(SH2_GYRO_INTEGRATED_RV);

  // Launch core 1
  multicore_launch_core1(core1Entry);

  // Power servo rail
  digitalWrite(6, HIGH);
  delay(500);

  // Set sitting pose
  applyPositions(sitPose);
  delay(2000);  // Allow robot to settle

  // Sample IMU to build pitch/roll calibration offsets
  float euler[3] = {0.0}, xSum = 0.0, ySum = 0.0;

  for (int i = 0; i < IMU_OFFSET_SAMPLES; i++) {
    updateIMU();
    float q[4] = {state.imu[0], state.imu[1], state.imu[2], state.imu[3]};
    quaternionToEuler(q, euler);

    xSum += euler[0];
    ySum += euler[1];
    delay(50);
  }

  calX = -(xSum / IMU_OFFSET_SAMPLES);
  calY = -(ySum / IMU_OFFSET_SAMPLES);

  // Stand up, then lock heading for drift correction
  standUp();
  lockHeading();

  timer = millis();
}


void core1Entry() {
  while (true) {
    updateDistanceReadings();
    updatePowerReading();
  }
}


void loop() {
  // Update sensors as fast as possible
  updateIMU();
  updateServoFeedback();

  // Run inference when the control period expires
  if (millis() - timer >= PERIOD) {
    timer = millis();

    // Current euler angles
    float euler[3] = {0.0};
    float q[4] = {state.imu[0], state.imu[1], state.imu[2], state.imu[3]};
    quaternionToEuler(q, euler);

    // Update yaw with locked heading
    euler[2] = fmodf((euler[2] - baseHeading) + M_PI, 2.0f * M_PI) - M_PI;

    // Build observations from the latest orientation and latest action
    buildObservations(euler, lastPositions);

    // Run distilled policy
    entry(infObs, infAct);
    float newActions[8] = {0.0};
    processActions(newActions);

    // Execute new actions & update last positions
    applyPositions(newActions);

    for (int i = 0; i < 8; i++) {
      lastPositions[i] = newActions[i];
    }
  }
}


//
// Inference functions
//
void buildObservations(const float euler[3], const float actions[8]) {
  // Latest orientation reading, scaled to [-1,1] (3 values)
  for (int i = 0; i < 3; i++) {
    float scaled = scaleValue(euler[i], -1.0f, 1.0f);
    infObs[0][i] = constrain(round4(scaled), -1.0f, 1.0f);
  }

  // Latest commanded action, scaled to [-1,1] (8 values)
  for (int i = 0; i < 8; i++) {
    float scaled = scaleValue(actions[i], servoDegMin[i], servoDegMax[i]);
    infObs[0][3 + i] = constrain(round4(scaled), -1.0f, 1.0f);
  }
}


void processActions(float newActions[8]) {
  for (int i = 0; i < 8; i++) {
    float delta = constrain(infAct[0][i], -3.0f, 3.0f) * (4.0f / 3.0f);
    newActions[i] = constrain(lastPositions[i] + delta, servoDegMin[i], servoDegMax[i]);
  }
}


void lockHeading() {
  float q[4] = {state.imu[0], state.imu[1], state.imu[2], state.imu[3]};
  float euler[3] = {0.0};

  quaternionToEuler(q, euler);
  baseHeading = euler[2];
}


//
// Sensor update functions
//
void updateIMU() {
  if (imu.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      state.imu[0] = imuValue.un.gyroIntegratedRV.real;
      state.imu[1] = imuValue.un.gyroIntegratedRV.i;
      state.imu[2] = imuValue.un.gyroIntegratedRV.j;
      state.imu[3] = imuValue.un.gyroIntegratedRV.k;
    }
  }
}


void updateDistanceReadings() {
  for (int i = 0; i < 4; i++) {
    mux.openChannel(i);
    uint16_t d = distSensors[i].readRangeContinuousMillimeters();

    if (distSensors[i].timeoutOccurred() || d >= 2000) {
      distBuffer[i] = 2000; // Max range (mm)
    } else {
      distBuffer[i] = d;
    }
    mux.closeChannel(i);
  }

  core1Writing = true;
  while (core0Reading) delay(0);
  for (int i = 0; i < 4; i++) state.dist[i] = distBuffer[i];
  core1Writing = false;
}


void updatePowerReading() {
  float v_avg = 0;

  for (int i = 0; i < 10; i++) {
    v_avg += analogRead(A0);
  }

  v_avg = 9.0 + ((v_avg / 10) - 2574.0) * 4.0 / (3721.0f - 2574.0);
  v_avg = constrain(v_avg, 9.0, 13.0);

  core1Writing = true;
  while (core0Reading) delay(0);
  state.power = (uint16_t)((v_avg - 9.0) / 4.0 * 65535.0);
  core1Writing = false;
}

void updateServoFeedback() {
  for (int i = 0; i < 8; i++) {
    if (servoDriver.FeedBack(i) != -1) {
      state.pos[i] = servoDriver.ReadPos(-1);
      state.speed[i] = servoDriver.ReadSpeed(-1);
      state.load[i] = servoDriver.ReadLoad(-1);
      state.voltage[i] = servoDriver.ReadVoltage(-1);
      state.current[i] = servoDriver.ReadCurrent(-1);
      state.temp[i] = servoDriver.ReadTemper(-1);
    }
  }
}


//
// Servo functions
//
void applyPositions(const float newPositions[8]) {
  int mapped[8] = {0};
  deg2servo(newPositions, mapped);

  for (int i = 0; i < 8; i++) {
    int pos = mapped[i];

    if (pos <= servoMin[i]){
      pos = servoMin[i];
    }
    else if (pos >= servoMax[i]) {
      pos = servoMax[i];
    }

    servoDriver.WritePosEx(i, pos, 3400, 0);
  }
}


void deg2servo(const float degrees[8], int mapped[8]) {
  int raw[8];
  for (int i = 0; i < 8; i++) {
    raw[i] = (int)(degrees[i] * 4095.0f / 360.0f);
  }
  mapped[0] = centers[0] - raw[0];  // RHip: negate
  mapped[1] = centers[1] + raw[1];  // LHip: same
  mapped[2] = centers[2] + raw[2];  // RShoulder: same
  mapped[3] = centers[3] + raw[3];  // LShoulder: same
  mapped[4] = centers[4] + raw[4];  // RKnee: same
  mapped[5] = centers[5] - raw[5];  // LKnee: negate
  mapped[6] = centers[6] - raw[6];  // RAnkle: negate
  mapped[7] = centers[7] + raw[7];  // LAnkle: same
}


//
// Math helpers
//
void quaternionToEuler(const float q[4], float euler[3]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float norm = sqrtf(w * w + x * x + y * y + z * z);

  if (norm < 1e-6f) {
    euler[0] = calX; euler[1] = calY; euler[2] = 0.0f;
    return;
  }

  w /= norm; x /= norm; y /= norm; z /= norm;

  float sinr = 2.0f * (w * x + y * z);
  float cosr = 1.0f - 2.0f * (x * x + y * y);
  euler[0] = atan2f(sinr, cosr) + calX;

  float sinp = 2.0f * (w * y - z * x);
  euler[1] = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI / 2.0f, sinp) : asinf(sinp);
  euler[1] += calY;

  float siny = 2.0f * (w * z + x * y);
  float cosy = 1.0f - 2.0f * (y * y + z * z);
  euler[2] = atan2f(siny, cosy);
}


float scaleValue(float v, float vmin, float vmax) {
  if (vmax == vmin) return 0.0f;
  return constrain((v - vmin) / (vmax - vmin) * 2.0f - 1.0f, -1.0f, 1.0f);
}


float round4(float v) {
  return roundf(v * 10000.0f) / 10000.0f;
}

//
// Hardcoded stand-up routine
//
void standUp() {
  float routine[][8] = {
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 115, 115, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 93, 93},
    {10, 10, 0, 0, 139, 139, 91, 91},
    {10, 10, 0, 0, 139, 139, 89, 89},
    {10, 10, 0, 0, 139, 139, 87, 87},
    {10, 10, 0, 0, 139, 139, 86, 86},
    {10, 10, 0, 0, 139, 139, 84, 84},
    {10, 10, 0, 0, 139, 139, 83, 83},
    {10, 10, 0, 0, 139, 139, 81, 81},
    {10, 10, 0, 0, 139, 139, 80, 80},
    {10, 10, 0, 0, 139, 139, 78, 78},
    {10, 10, 0, 0, 139, 139, 77, 77},
    {10, 10, 0, 0, 139, 139, 75, 75},
    {10, 10, 0, 0, 139, 139, 74, 74},
    {10, 10, 0, 0, 139, 139, 72, 72},
    {10, 10, 0, 0, 139, 139, 71, 71},
    {10, 10, 0, 0, 139, 139, 69, 69},
    {10, 10, 0, 0, 139, 139, 68, 68},
    {10, 10, 0, 0, 139, 139, 66, 66},
    {10, 10, 0, 0, 139, 139, 65, 65},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-45, -45, 0, 0, 139, 139, 64, 64},
    {-43, -43, 0, 0, 129, 129, 62, 62},
    {-42, -42, 0, 0, 124, 124, 60, 60},
    {-41, -41, 0, 0, 119, 119, 58, 58},
    {-40, -40, 0, 0, 114, 114, 56, 56},
    {-39, -39, 0, 0, 109, 109, 54, 54},
    {-38, -38, 0, 0, 104, 104, 52, 52},
    {-37, -37, 0, 0, 99, 99, 50, 50},
    {-36, -36, 0, 0, 94, 94, 48, 48},
    {-35, -35, 0, 0, 89, 89, 46, 46},
    {-34, -34, 0, 0, 84, 84, 44, 44},
    {-33, -33, 0, 0, 79, 79, 42, 42},
    {-32, -32, 0, 0, 74, 74, 40, 40},
    {-31, -31, 0, 0, 69, 69, 38, 38},
    {-30, -30, 0, 0, 66, 66, 36, 36},
    {-30, -30, 0, 0, 64, 64, 34, 34},
    {-30, -30, 0, 0, 62, 62, 32, 32},
    {-30, -30, 0, 0, 60, 60, 30, 30}
  };

  int steps = sizeof(routine) / sizeof(routine[0]);
  for (int i = 0; i < steps; i++) {
    applyPositions(routine[i]);
    delay(49);
  }
}
