#pragma once

#include <stdint.h>

enum class GyroRead { Ok, Pending, Saturated, Unavailable };
enum class ImuInitResult { Pending, Ready, InitError, CalibrationError };

namespace icm20602
{
constexpr uint8_t kGyro250Dps = 0x00;
constexpr float kCountsPerDegPerSec = 131.0f;
constexpr int32_t kGyroRailCounts = 32760;

constexpr int32_t decodeRawZ(uint8_t high, uint8_t low)
{
    const int32_t word = (static_cast<int32_t>(high) << 8) | low;
    return word >= 32768 ? word - 65536 : word;
}

constexpr bool atRail(int32_t raw)
{
    return raw <= -kGyroRailCounts || raw >= kGyroRailCounts;
}

constexpr float toDegPerSec(int32_t raw)
{
    return raw / kCountsPerDegPerSec;
}
}

namespace imu
{
bool begin();
ImuInitResult updateInit();
GyroRead readGyroZ(float& rateDegPerSec);
}
