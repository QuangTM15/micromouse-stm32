#include "encoder.h"

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

void encoderLeftISR()
{
  encoderLeft++;
}

void encoderRightISR()
{
  encoderRight++;
}

void readEncoders(long &left, long &right)
{
  noInterrupts();
  left = encoderLeft;
  right = encoderRight;
  interrupts();
}

void resetEncoders()
{
  noInterrupts();
  encoderLeft = 0;
  encoderRight = 0;
  interrupts();
}
