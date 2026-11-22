#pragma once

#include <stdint.h>

enum class TofPosition { Right, Left, Front };
enum class TofResult { Ready, Pending, Timeout, BadRange, BusError, Unavailable };
enum class TofInitResult { Off, Pending, Ready, Failed };

namespace tof
{
// Initialize after I2C setup. Service only with motion disarmed.
void begin();
TofInitResult updateInit();
void shutdown();
// Foreground only. Writes millimeters only on Ready.
TofResult poll(TofPosition position, uint16_t& millimeters);
}
