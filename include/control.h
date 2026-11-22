#pragma once

#include <stdint.h>
#include "motion_config.h"

struct EncoderCounts
{
    uint32_t left = 0;
    uint32_t right = 0;
};

struct WheelPwm
{
    int left = 0;
    int right = 0;
};

struct PidState
{
    float integral = 0;
};

struct WheelMonitor
{
    uint32_t lastProgressUs = 0;
    int64_t wrongCounts = 0;
};

enum class WheelResult { Ok, Stalled, WrongDirection };

constexpr int encoderEdgeDelta(bool bHigh, int sign)
{
    return bHigh ? -sign : sign;
}

int64_t countDelta(uint32_t current, uint32_t previous);
int direction(float value);
float wheelBalancePwm(float leftMmPerSec, float rightMmPerSec, float basePwm);
float pidUpdate(PidState& state, const PidGains& gains, float error, float errorRate,
                float dtSec, float lower, float upper, bool integrate = true);
float distanceDemand(float remainingMm);
float turnDemand(PidState& state, float errorDeg, float rateDegPerSec, float dtSec);
WheelResult monitorWheel(WheelMonitor& monitor, uint32_t nowUs, int previousPwm, int64_t delta);
float slewPwm(float current, float target, float dtSec);
