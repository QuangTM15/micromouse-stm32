#pragma once
#include "config.h"
#include <VL53L0X.h>
extern VL53L0X sensorFront, sensorLeft, sensorRight;
extern uint16_t currentDistL, currentDistR;
bool initSensor(VL53L0X &, uint8_t, uint8_t);
void setupSensors();
