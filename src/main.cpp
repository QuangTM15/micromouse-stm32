
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"

// =====================================================
// MICROMOUSE - HEADING PID TEST
// STM32F411CEU6 + DRV8833 + ICM-20602
//
// Power ON -> Wait 2s
// Straight PID 2s
// Pause 1s
// Turn LEFT ~90 degrees
// Pause 1s
// Straight PID 2s
// STOP
// =====================================================

// ================= MOTOR CONFIG =================

static const int BASE_PWM = 120;
static const int TURN_PWM = 120;

static const int MIN_PWM = 100;
static const int MAX_PWM = 150;

static const float HEADING_MAX_CORRECTION = 20.0f;

// ================= HEADING PID =================

// Start with conservative gains.
// Units:
// KP: PWM / degree
// KI: PWM / (degree * second)
// KD: PWM / (degree / second)

static const float HEADING_KP = 2.0f;
static const float HEADING_KI = 0.06f;
static const float HEADING_KD = 0.15f;

static const float MAX_INTEGRAL = 25.0f;

// IMPORTANT:
// +1 = positive gyro Z means physical LEFT turn.
// -1 = negative gyro Z means physical LEFT turn.
//
// Change to -1 if PID makes left drift worse.
static const float IMU_LEFT_SIGN = 1.0f;

// ================= TIMING =================

static const uint32_t START_DELAY_MS = 2000U;
static const uint32_t STRAIGHT_MS = 2000U;
static const uint32_t PAUSE_MS = 1000U;

static const uint32_t CONTROL_MS = 20U;
static const uint32_t TURN_TIMEOUT_MS = 3000U;
static const uint32_t TURN_DETECT_MS = 600U;

// ================= TURN CONFIG =================

static const float TURN_STOP_DEG = 85.0f;
static const float GYRO_DEADBAND = 1.0f;

// ================= SAFETY =================

static const float MAX_HEADING_ERROR = 35.0f;
static const uint32_t STALL_TIMEOUT_MS = 600U;

// ================= STATE =================

enum TestState
{
  WAIT_START,
  STRAIGHT_1,
  PAUSE_1,
  TURN_LEFT,
  PAUSE_2,
  STRAIGHT_2,
  FINISHED,
  ERROR_STOP
};

static TestState state = WAIT_START;

static uint32_t stateStartMs = 0U;
static uint32_t lastControlMs = 0U;
static uint32_t lastGyroUs = 0U;

// ================= HEADING PID =================

static float headingAngle = 0.0f;
static float headingIntegral = 0.0f;
static float filteredGyro = 0.0f;

// ================= TURN =================

static float turnAngle = 0.0f;
static bool turnDetected = false;

// ================= ENCODER WATCHDOG =================

static long previousLeftTicks = 0L;
static long previousRightTicks = 0L;

static uint32_t leftMovementMs = 0U;
static uint32_t rightMovementMs = 0U;

// =====================================================
// DEBUG
// =====================================================

void debugMessage(const char *message)
{
  if (Serial)
  {
    Serial.println(message);
  }
}

void printStraightResult()
{
  if (!Serial)
  {
    return;
  }

  long left = 0L;
  long right = 0L;

  readEncoders(left, right);

  Serial.print("[RESULT] L=");
  Serial.print(left);

  Serial.print(" R=");
  Serial.print(right);

  Serial.print(" Heading=");
  Serial.print(headingAngle, 2);

  Serial.println(" deg");
}

// =====================================================
// MOTOR
// =====================================================

void initializeMotor()
{
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(MOTOR_SLEEP, OUTPUT);

  digitalWrite(MOTOR_SLEEP, LOW);

  motorCoast();
}

void stopMotor()
{
  motorCoast();
  digitalWrite(MOTOR_SLEEP, LOW);
}

void driveMotor(int leftPWM, int rightPWM)
{
  digitalWrite(MOTOR_SLEEP, HIGH);

  setMotors(leftPWM, rightPWM);
}

// =====================================================
// ENCODER
// =====================================================

void initializeEncoder()
{
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);

  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  resetEncoders();

  attachInterrupt(
      digitalPinToInterrupt(ENC_LEFT_A),
      encoderLeftISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(ENC_RIGHT_A),
      encoderRightISR,
      RISING);
}

// =====================================================
// IMU
// =====================================================

bool initializeIMU()
{
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setSDA(I2C_SDA_PIN);

  Wire.begin();
  Wire.setClock(50000);

  Wire.beginTransmission(ICM_ADDR);

  if (Wire.endTransmission() != 0)
  {
    return false;
  }

  // Existing setupImu() calibrates gyro offset.
  // Keep the robot completely stationary here.
  setupImu();

  return true;
}

// =====================================================
// RESET HEADING PID
// =====================================================

void resetHeadingPID()
{
  headingAngle = 0.0f;
  headingIntegral = 0.0f;
  filteredGyro = 0.0f;

  lastGyroUs = micros();

  resetEncoders();

  previousLeftTicks = 0L;
  previousRightTicks = 0L;

  leftMovementMs = millis();
  rightMovementMs = millis();
}

// =====================================================
// STATE MACHINE
// =====================================================

void enterState(TestState next)
{
  if (state == STRAIGHT_1 ||
      state == STRAIGHT_2)
  {
    printStraightResult();
  }

  stopMotor();

  state = next;
  stateStartMs = millis();
  lastControlMs = stateStartMs;

  switch (next)
  {
  case WAIT_START:
    debugMessage("[WAIT] 2 seconds");
    break;

  case STRAIGHT_1:
    resetHeadingPID();

    driveMotor(BASE_PWM, BASE_PWM);

    debugMessage("[1] STRAIGHT PID");
    break;

  case PAUSE_1:
    debugMessage("[2] PAUSE");
    break;

  case TURN_LEFT:
    resetEncoders();

    turnAngle = 0.0f;
    turnDetected = false;
    lastGyroUs = micros();

    driveMotor(-TURN_PWM, TURN_PWM);

    debugMessage("[3] TURN LEFT");
    break;

  case PAUSE_2:
    debugMessage("[4] PAUSE");
    break;

  case STRAIGHT_2:
    resetHeadingPID();

    driveMotor(BASE_PWM, BASE_PWM);

    debugMessage("[5] STRAIGHT PID");
    break;

  case FINISHED:
    debugMessage("[DONE] TEST FINISHED");
    break;

  case ERROR_STOP:
    debugMessage("[ERROR] ROBOT STOPPED");
    break;
  }
}

// =====================================================
// STRAIGHT HEADING PID
// =====================================================

void updateHeadingPID()
{
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - stateStartMs) >= STRAIGHT_MS)
  {
    if (state == STRAIGHT_1)
    {
      enterState(PAUSE_1);
    }
    else
    {
      enterState(FINISHED);
    }

    return;
  }

  if ((uint32_t)(nowMs - lastControlMs) < CONTROL_MS)
  {
    return;
  }

  lastControlMs = nowMs;

  // ================= DELTA TIME =================

  uint32_t nowUs = micros();

  float dt =
      (float)(uint32_t)(nowUs - lastGyroUs) /
      1000000.0f;

  lastGyroUs = nowUs;

  if (dt <= 0.0f || dt > 0.15f)
  {
    enterState(ERROR_STOP);
    return;
  }

  // ================= READ GYRO =================

  float gyroZ =
      getGyroZRate() * IMU_LEFT_SIGN;

  if (!isfinite(gyroZ))
  {
    enterState(ERROR_STOP);
    return;
  }

  // Low-pass filter gyro velocity.
  filteredGyro =
      0.65f * filteredGyro +
      0.35f * gyroZ;

  if (fabsf(filteredGyro) < GYRO_DEADBAND)
  {
    filteredGyro = 0.0f;
  }

  // ================= ESTIMATE HEADING =================

  // Target heading = 0 degrees.
  headingAngle += filteredGyro * dt;

  if (fabsf(headingAngle) > MAX_HEADING_ERROR)
  {
    debugMessage("[ERROR] HEADING TOO LARGE");
    enterState(ERROR_STOP);
    return;
  }

  // ================= INTEGRAL =================

  if (fabsf(headingAngle) < 15.0f)
  {
    headingIntegral += headingAngle * dt;

    headingIntegral = constrain(
        headingIntegral,
        -MAX_INTEGRAL,
        MAX_INTEGRAL);
  }

  // ================= PID =================

  float proportional =
      HEADING_KP * headingAngle;

  float integral =
      HEADING_KI * headingIntegral;

  // Gyro rate is derivative of heading.
  float derivative =
      HEADING_KD * filteredGyro;

  float correction =
      proportional + integral + derivative;

  correction = constrain(
      correction,
      -HEADING_MAX_CORRECTION,
      HEADING_MAX_CORRECTION);

  // If robot turns left:
  // increase LEFT wheel,
  // decrease RIGHT wheel.
  int leftPWM = constrain(
      (int)lroundf(BASE_PWM + correction),
      MIN_PWM,
      MAX_PWM);

  int rightPWM = constrain(
      (int)lroundf(BASE_PWM - correction),
      MIN_PWM,
      MAX_PWM);

  // ================= ENCODER SAFETY =================

  long leftTicks = 0L;
  long rightTicks = 0L;

  readEncoders(leftTicks, rightTicks);

  if (leftTicks != previousLeftTicks)
  {
    previousLeftTicks = leftTicks;
    leftMovementMs = nowMs;
  }

  if (rightTicks != previousRightTicks)
  {
    previousRightTicks = rightTicks;
    rightMovementMs = nowMs;
  }

  if ((uint32_t)(nowMs - leftMovementMs) >= STALL_TIMEOUT_MS ||
      (uint32_t)(nowMs - rightMovementMs) >= STALL_TIMEOUT_MS)
  {
    debugMessage("[ERROR] ENCODER STALL");
    enterState(ERROR_STOP);
    return;
  }

  // ================= APPLY PWM =================

  driveMotor(leftPWM, rightPWM);
}

// =====================================================
// TURN LEFT 90 DEGREES
// =====================================================

void updateTurnLeft()
{
  uint32_t nowMs = millis();

  uint32_t elapsedMs = nowMs - stateStartMs;

  if (elapsedMs >= TURN_TIMEOUT_MS)
  {
    debugMessage("[ERROR] TURN TIMEOUT");
    enterState(ERROR_STOP);
    return;
  }

  if ((uint32_t)(nowMs - lastControlMs) < CONTROL_MS)
  {
    return;
  }

  lastControlMs = nowMs;

  uint32_t nowUs = micros();

  float dt =
      (float)(uint32_t)(nowUs - lastGyroUs) /
      1000000.0f;

  lastGyroUs = nowUs;

  if (dt <= 0.0f || dt > 0.15f)
  {
    enterState(ERROR_STOP);
    return;
  }

  float gyroZ = getGyroZRate();

  if (!isfinite(gyroZ))
  {
    enterState(ERROR_STOP);
    return;
  }

  float angularSpeed = fabsf(gyroZ);

  if (angularSpeed < GYRO_DEADBAND)
  {
    angularSpeed = 0.0f;
  }

  if (angularSpeed >= 5.0f)
  {
    turnDetected = true;
  }

  if (elapsedMs >= TURN_DETECT_MS && !turnDetected)
  {
    debugMessage("[ERROR] NO TURN DETECTED");
    enterState(ERROR_STOP);
    return;
  }

  turnAngle += angularSpeed * dt;

  if (turnAngle >= TURN_STOP_DEG)
  {
    enterState(PAUSE_2);
    return;
  }

  driveMotor(-TURN_PWM, TURN_PWM);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  // Disable motor immediately.
  initializeMotor();

  Serial.begin(115200);

  initializeEncoder();

  if (!initializeIMU())
  {
    stopMotor();
    state = ERROR_STOP;

    debugMessage("[ERROR] IMU NOT DETECTED");
    return;
  }

  stopMotor();

  debugMessage("==========================");
  debugMessage(" MICROMOUSE HEADING PID");
  debugMessage("==========================");

  // Countdown starts after IMU calibration.
  enterState(WAIT_START);
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  uint32_t nowMs = millis();

  switch (state)
  {
  case WAIT_START:
    if ((uint32_t)(nowMs - stateStartMs) >=
        START_DELAY_MS)
    {
      enterState(STRAIGHT_1);
    }
    break;

  case STRAIGHT_1:
  case STRAIGHT_2:
    updateHeadingPID();
    break;

  case PAUSE_1:
    if ((uint32_t)(nowMs - stateStartMs) >= PAUSE_MS)
    {
      enterState(TURN_LEFT);
    }
    break;

  case TURN_LEFT:
    updateTurnLeft();
    break;

  case PAUSE_2:
    if ((uint32_t)(nowMs - stateStartMs) >= PAUSE_MS)
    {
      enterState(STRAIGHT_2);
    }
    break;

  case FINISHED:
  case ERROR_STOP:
  default:
    // Motors already disabled by enterState().
    break;
  }
}
