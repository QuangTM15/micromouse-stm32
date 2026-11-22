#include "imu.h"

#include <algorithm>
#include <cmath>
#include "config.h"
#include "i2c_bus.h"

namespace
{
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kDeviceId = 0x12;
constexpr uint8_t kPowerManagement = 0x6B;
constexpr uint8_t kAxisStandby = 0x6C;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kFilterConfig = 0x1A;
constexpr uint8_t kGyroFilter = 0x02; // 92 Hz DLPF, 1 kHz output.
constexpr uint8_t kInterruptPinConfig = 0x37;
constexpr uint8_t kLatchInterrupt = 0x20;
constexpr uint8_t kInterruptEnable = 0x38;
constexpr uint8_t kInterruptStatus = 0x3A;
constexpr uint8_t kDataReady = 0x01;
constexpr uint8_t kGyroZHigh = 0x47;
constexpr uint8_t kAutoClock = 0x01;
using icm20602::kGyro250Dps;

enum class Phase { Off, Resetting, Stabilizing, Calibrating, Ready };
Phase phase = Phase::Off;
uint32_t phaseStartMs = 0;
uint32_t sampleMs = 0;
uint16_t samples = 0;
float sum = 0;
float minRate = 250.0f;
float maxRate = -250.0f;
float bias = 0;

bool matches(uint8_t reg, uint8_t expected)
{
    uint8_t value = 0;
    return i2c::read(board::imuAddress, reg, &value, 1) && value == expected;
}

bool configure()
{
    return matches(kWhoAmI, kDeviceId) &&
        i2c::write(board::imuAddress, kPowerManagement, kAutoClock) &&
        i2c::write(board::imuAddress, kAxisStandby, 0) &&
        i2c::write(board::imuAddress, kGyroConfig, kGyro250Dps) &&
        i2c::write(board::imuAddress, kFilterConfig, kGyroFilter) &&
        i2c::write(board::imuAddress, kInterruptPinConfig, kLatchInterrupt) &&
        i2c::write(board::imuAddress, kInterruptEnable, kDataReady) &&
        matches(kPowerManagement, kAutoClock) && matches(kGyroConfig, kGyro250Dps) &&
        matches(kFilterConfig, kGyroFilter);
}

GyroRead readRaw(float& rate)
{
    uint8_t bytes[kGyroZHigh - kInterruptStatus + 2]{};
    if (!i2c::read(board::imuAddress, kInterruptStatus, bytes, sizeof(bytes)))
        return GyroRead::Unavailable;
    if ((bytes[0] & kDataReady) == 0)
        return GyroRead::Pending;

    constexpr size_t index = kGyroZHigh - kInterruptStatus;
    const int32_t raw = icm20602::decodeRawZ(bytes[index], bytes[index + 1]);
    if (icm20602::atRail(raw))
        return GyroRead::Saturated;
    rate = icm20602::toDegPerSec(raw);
    return GyroRead::Ok;
}
}

bool imu::begin()
{
    if (!matches(kWhoAmI, kDeviceId) || !i2c::write(board::imuAddress, kPowerManagement, 0x80))
        return false;

    phaseStartMs = millis();
    phase = Phase::Resetting;
    samples = 0;
    minRate = 250;
    maxRate = -250;
    sum = bias = 0;
    return true;
}

ImuInitResult imu::updateInit()
{
    const uint32_t now = millis();
    switch (phase)
    {
    case Phase::Off:
        return ImuInitResult::InitError;
    case Phase::Resetting:
        if (now - phaseStartMs >= board::imuResetMs)
        {
            if (!configure())
                return ImuInitResult::InitError;
            phaseStartMs = millis();
            phase = Phase::Stabilizing;
        }
        return ImuInitResult::Pending;
    case Phase::Stabilizing:
        if (now - phaseStartMs >= board::imuStabilizeMs)
        {
            phaseStartMs = sampleMs = now;
            phase = Phase::Calibrating;
        }
        return ImuInitResult::Pending;
    case Phase::Ready:
        return ImuInitResult::Ready;
    case Phase::Calibrating:
        break;
    }

    if (now - phaseStartMs >= board::gyroCalibrationTimeoutMs)
        return ImuInitResult::CalibrationError;
    if (now - sampleMs < board::gyroCalibrationIntervalMs)
        return ImuInitResult::Pending;
    sampleMs = now;
    float rate = 0;
    const auto result = readRaw(rate);
    if (result == GyroRead::Pending)
        return ImuInitResult::Pending;
    if (result != GyroRead::Ok)
        return ImuInitResult::CalibrationError;

    sum += rate;
    minRate = std::min(minRate, rate);
    maxRate = std::max(maxRate, rate);
    if (maxRate - minRate > board::gyroCalibrationMaxSpanDegPerSec)
        return ImuInitResult::CalibrationError;
    if (++samples < board::gyroCalibrationSamples)
        return ImuInitResult::Pending;

    bias = sum / samples;
    if (std::fabs(bias) > board::gyroCalibrationMaxBiasDegPerSec)
        return ImuInitResult::CalibrationError;
    phase = Phase::Ready;
    return ImuInitResult::Ready;
}

GyroRead imu::readGyroZ(float& rateDegPerSec)
{
    if (phase != Phase::Ready)
        return GyroRead::Unavailable;
    const auto result = readRaw(rateDegPerSec);
    if (result == GyroRead::Ok)
        rateDegPerSec -= bias;
    return result;
}
