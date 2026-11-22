#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "imu.h"
#include "motion.h"
#include "maze.h"
#include "robot.h"



bool driveStraight(float distance_cm)
{
  float distance_mm = distance_cm * 10.0f;
  long targetCounts = (long)(distance_mm / MM_PER_COUNT);

  resetEncoders();

  float yaw = 0.0f;
  unsigned long lastTime = micros();
  unsigned long startTime = millis();

  while (true)
  {
    long leftCount, rightCount;
    readEncoders(leftCount, rightCount);

    if ((leftCount + rightCount) / 2 >= targetCounts)
    {
      setMotors(0, 0);
      delay(100);
      return true;
    }

    if (millis() - startTime > 10000)
    {
      setMotors(0, 0);
      delay(100);
      Serial.println("[LOI] Qua thoi gian khi di mot o!");
      return false;
    }

    uint16_t distFront =
        sensorFront.readRangeContinuousMillimeters();
    bool frontTimeout = sensorFront.timeoutOccurred();

    if (!frontTimeout &&
        distFront > 20 &&
        distFront <= IDEAL_FRONT_DIST_MM)
    {
      setMotors(0, 0);
      delay(100);
      Serial.println(
          "[DUNG AN TOAN] Phat hien tuong sat phia truoc.");
      return false;
    }

    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    yaw += getGyroZRate() * dt;

    // IMU correction theo dithang_IMU.ino.
    float error = yaw;
    int imuCorrection = constrain(
        (int)(error * IMU_KP),
        -MAX_CORRECTION,
        MAX_CORRECTION);

    uint16_t distL =
        sensorLeft.readRangeContinuousMillimeters();
    bool leftTimeout = sensorLeft.timeoutOccurred();

    uint16_t distR =
        sensorRight.readRangeContinuousMillimeters();
    bool rightTimeout = sensorRight.timeoutOccurred();

    bool leftWall =
        !leftTimeout &&
        distL > 20 &&
        distL < WALL_THRESHOLD_MM;

    bool rightWall =
        !rightTimeout &&
        distR > 20 &&
        distR < WALL_THRESHOLD_MM;

    float wallCorrection = 0.0f;

    if (leftWall && rightWall)
    {
      wallCorrection =
          ((float)distR - (float)distL) * WALL_FOLLOW_KP;
    }
    else if (leftWall)
    {
      wallCorrection =
          (SIDE_WALL_TARGET_MM - (float)distL) * WALL_FOLLOW_KP;
    }
    else if (rightWall)
    {
      wallCorrection =
          ((float)distR - SIDE_WALL_TARGET_MM) * WALL_FOLLOW_KP;
    }

    wallCorrection = constrain(
        wallCorrection,
        -MAX_WALL_CORRECTION,
        MAX_WALL_CORRECTION);

    int correction = constrain(
        imuCorrection + (int)wallCorrection,
        -MAX_CORRECTION,
        MAX_CORRECTION);

    setMotors(
        BASE_PWM_STRAIGHT + correction,
        BASE_PWM_STRAIGHT - correction);
  }
}

bool executeTurn(float targetAngle, bool turnLeft)
{
  resetEncoders();
  motorCoast();
  delay(20);

  float slowdownAngle =
      (targetAngle > 100) ? 60.0f : 30.0f;
  float inertiaOffset =
      (targetAngle > 100) ? 8.0f : 3.0f;
  uint32_t timeoutMs =
      (targetAngle > 100) ? 5000 : 4000;

  float currentAngle = 0.0f;
  uint32_t startMs = millis();
  uint32_t lastMicros = micros();

  Serial.print(" -> XOAY ");
  Serial.println(turnLeft ? "TRAI" : "PHAI");

  while (true)
  {
    uint32_t currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;

    currentAngle +=
        abs(getGyroZRate() * dt) * IMU_SCALE_FACTOR;

    if (currentAngle >= (targetAngle - inertiaOffset))
    {
      motorBrake();
      delay(200);
      return true;
    }

    if (millis() - startMs >= timeoutMs)
    {
      motorBrake();
      delay(200);
      Serial.println("[LOI] Ket xe khi dang quay!");
      return false;
    }

    long leftCount, rightCount;
    readEncoders(leftCount, rightCount);

    long errorCounts = leftCount - rightCount;
    int correction = constrain(
        (int)roundf(TURN_BALANCE_KP * (float)errorCounts),
        -MAX_TURN_CORRECTION,
        MAX_TURN_CORRECTION);

    uint32_t elapsedMs = millis() - startMs;
    float centerDuty = (float)BASE_TURN_DUTY;

    if (elapsedMs < START_BOOST_MS)
    {
      float boostFactor =
          1.0f - (float)elapsedMs / (float)START_BOOST_MS;
      centerDuty += START_BOOST_ADD * boostFactor;
    }

    float remainingAngle = targetAngle - currentAngle;

    if (remainingAngle < slowdownAngle)
    {
      float ratio = remainingAngle / slowdownAngle;
      float slowDuty =
          MIN_TURN_DUTY +
          (BASE_TURN_DUTY - MIN_TURN_DUTY) * ratio;

      if (slowDuty < centerDuty)
      {
        centerDuty = slowDuty;
      }
    }

    int dutyLeft = constrain(
        (int)roundf(centerDuty - correction),
        MIN_TURN_DUTY,
        MAX_TURN_DUTY);

    int dutyRight = constrain(
        (int)roundf(centerDuty + correction),
        MIN_TURN_DUTY,
        MAX_TURN_DUTY);

    updateTurnPWM(turnLeft, dutyLeft, dutyRight);
  }
}

bool executeTurn180(bool turnLeft)
{
  resetEncoders();
  motorCoast();
  delay(20);

  float currentAngle = 0.0f;
  uint32_t startMs = millis();
  uint32_t lastMicros = micros();

  while (true)
  {
    uint32_t currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;

    currentAngle +=
        abs(getGyroZRate() * dt) * TURN180_IMU_SCALE_FACTOR;

    // Không bù góc, không ngắt sớm: đợi IMU đo đủ 180 độ.
    if (currentAngle >= TURN180_TARGET_ANGLE)
    {
      break;
    }

    if (millis() - startMs >= TURN180_TIMEOUT_MS)
    {
      motorBrake();
      delay(200);
      Serial.println("[LOI] Quay 180 do qua thoi gian!");
      return false;
    }

    long leftCount, rightCount;
    readEncoders(leftCount, rightCount);

    long errorCounts = leftCount - rightCount;
    int correction = (int)roundf(
        TURN180_BALANCE_KP * (float)errorCounts);

    correction = constrain(
        correction,
        -TURN180_MAX_CORRECTION,
        TURN180_MAX_CORRECTION);

    uint32_t elapsedMs = millis() - startMs;
    float centerDuty = (float)TURN180_BASE_DUTY;

    if (elapsedMs < TURN180_START_BOOST_MS)
    {
      float boostFactor =
          1.0f -
          (float)elapsedMs / (float)TURN180_START_BOOST_MS;
      centerDuty += TURN180_START_BOOST_ADD * boostFactor;
    }

    float remainingAngle = TURN180_TARGET_ANGLE - currentAngle;

    if (remainingAngle < TURN180_SLOWDOWN_ANGLE)
    {
      float ratio =
          remainingAngle / TURN180_SLOWDOWN_ANGLE;

      float slowDuty =
          TURN180_MIN_DUTY +
          (TURN180_BASE_DUTY - TURN180_MIN_DUTY) * ratio;

      if (slowDuty < centerDuty)
      {
        centerDuty = slowDuty;
      }
    }

    int dutyLeft = constrain(
        (int)roundf(centerDuty - correction),
        TURN180_MIN_DUTY,
        TURN180_MAX_DUTY);

    int dutyRight = constrain(
        (int)roundf(centerDuty + correction),
        TURN180_MIN_DUTY,
        TURN180_MAX_DUTY);

    updateTurnPWM(turnLeft, dutyLeft, dutyRight);
  }

  motorBrake();
  delay(200);
  return true;
}

void reverseWithHeadingHold(int basePWM, uint32_t durationMs,
                            float &yaw, uint32_t &lastMicros,
                            bool contactAssist)
{
  uint32_t startMs = millis();

  while (millis() - startMs < durationMs)
  {
    uint32_t nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    lastMicros = nowMicros;

    yaw += getGyroZRate() * dt;

    int correction = 0;

    if (fabsf(yaw) > BACK_ALIGN_YAW_DEADBAND)
    {
      float kp = contactAssist
                     ? BACK_ALIGN_CONTACT_KP
                     : BACK_ALIGN_IMU_KP;

      int maxCorrection = contactAssist
                              ? BACK_ALIGN_CONTACT_MAX_CORRECTION
                              : BACK_ALIGN_MAX_CORRECTION;

      correction = constrain(
          (int)lroundf(yaw * kp),
          -maxCorrection,
          maxCorrection);
    }

    setMotors(
        basePWM + correction,
        basePWM - correction);

    delay(5);
  }
}

void alignWithWallBehind()
{
  bool wallBehind = false;
  int backDir = (currentDir + 2) % 4;

  if (backDir == 0)
  {
    wallBehind = maze[currentX][currentY].wallNorth;
  }
  else if (backDir == 1)
  {
    wallBehind = maze[currentX][currentY].wallEast;
  }
  else if (backDir == 2)
  {
    wallBehind = maze[currentX][currentY].wallSouth;
  }
  else if (backDir == 3)
  {
    wallBehind = maze[currentX][currentY].wallWest;
  }

  if (!wallBehind)
  {
    Serial.println(
        " -> KHONG CAN CHINH: Phia sau trong, tiep tuc di tiep.");
    return;
  }

  Serial.println(
      " -> CAN CHINH: Lui manh va giu huong bang IMU...");

  float reverseYaw = 0.0f;
  uint32_t lastGyroMicros = micros();

  reverseWithHeadingHold(
      BACK_ALIGN_PWM_APPROACH,
      BACK_ALIGN_APPROACH_MS,
      reverseYaw,
      lastGyroMicros,
      false);

  reverseWithHeadingHold(
      BACK_ALIGN_PWM_HOLD,
      BACK_ALIGN_HOLD_MS,
      reverseYaw,
      lastGyroMicros,
      true);

  motorBrake();
  delay(250);

  long targetCounts =
      (long)((BACK_ALIGN_OFFSET_CM * 10.0f) / MM_PER_COUNT);

  resetEncoders();
  unsigned long startMs = millis();

  while (true)
  {
    long leftCount, rightCount;
    readEncoders(leftCount, rightCount);

    if ((leftCount + rightCount) / 2 >= targetCounts)
      break;
    if (millis() - startMs > 1500)
      break;

    setMotors(70, 70);
  }

  motorBrake();
  delay(200);
}
