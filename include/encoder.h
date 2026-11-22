#pragma once
#include "config.h"
extern volatile long encoderLeft, encoderRight;
void encoderLeftISR();
void encoderRightISR();
void readEncoders(long &, long &);
void resetEncoders();
