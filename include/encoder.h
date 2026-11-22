#pragma once

#include "control.h"

namespace encoders
{
bool begin();
EncoderCounts read();
}
