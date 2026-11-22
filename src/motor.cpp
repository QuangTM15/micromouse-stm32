#include "motor.h"

void setOneMotor(uint8_t in1, uint8_t in2, int speed,
                 bool reverseMotor)
{
  speed = constrain(speed, -255, 255);

  if (reverseMotor)
  {
    speed = -speed;
  }

  if (speed > 0)
  {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  }
  else if (speed < 0)
  {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  }
  else
  {
    analogWrite(in1, 255);
    analogWrite(in2, 255);
  }
}

void setMotors(int leftPWM, int rightPWM)
{
  setOneMotor(
      LEFT_IN1, LEFT_IN2, leftPWM, REVERSE_LEFT_MOTOR);
  setOneMotor(
      RIGHT_IN1, RIGHT_IN2, rightPWM, REVERSE_RIGHT_MOTOR);
}

void motorCoast()
{
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}

void motorBrake()
{
  analogWrite(LEFT_IN1, 255);
  analogWrite(LEFT_IN2, 255);
  analogWrite(RIGHT_IN1, 255);
  analogWrite(RIGHT_IN2, 255);
}

void updateTurnPWM(bool turnLeft, int dutyLeft, int dutyRight)
{
  int pwmLeft = map(
      constrain(dutyLeft, 0, 100), 0, 100, 0, 255);
  int pwmRight = map(
      constrain(dutyRight, 0, 100), 0, 100, 0, 255);

  if (turnLeft)
  {
    analogWrite(LEFT_IN1, pwmLeft);
    analogWrite(LEFT_IN2, 0);
    analogWrite(RIGHT_IN1, pwmRight);
    analogWrite(RIGHT_IN2, 0);
  }
  else
  {
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, pwmLeft);
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, pwmRight);
  }
}
