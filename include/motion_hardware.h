#pragma once

#include "motion.h"

namespace hardware
{
MotionFault begin();
MotionFault updateInit(bool& ready);
uint32_t nowUs();
void serviceWatchdog();
}
