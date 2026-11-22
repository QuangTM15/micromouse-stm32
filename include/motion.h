#pragma once
#include "config.h"
bool driveStraight(float);
bool executeTurn(float, bool);
bool executeTurn180(bool);
void reverseWithHeadingHold(int, uint32_t, float &, uint32_t &, bool);
void alignWithWallBehind();
