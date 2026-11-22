#pragma once
#include "config.h"
void setOneMotor(uint8_t, uint8_t, int, bool);
void setMotors(int, int);
void motorCoast();
void motorBrake();
void updateTurnPWM(bool, int, int);
