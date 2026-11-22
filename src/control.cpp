#include "control.h"

#include <algorithm>
#include <cmath>

namespace
{
float approachFraction(float error, float tolerance)
{
    return std::clamp((error - tolerance) / tolerance, 0.0f, 1.0f);
}
}

int64_t countDelta(uint32_t current, uint32_t previous)
{
    const uint32_t difference = current - previous;
    if (difference <= INT32_MAX)
        return difference;
    return static_cast<int64_t>(difference) - (INT64_C(1) << 32);
}

int direction(float value)
{
    return (value > 0) - (value < 0);
}

float wheelBalancePwm(float leftMmPerSec, float rightMmPerSec, float basePwm)
{
    // The ratio is unitless. One encoder count has a different travel on each wheel.
    const float speedSum = std::fabs(leftMmPerSec) + std::fabs(rightMmPerSec);
    if (speedSum == 0)
        return 0;
    const float trim = std::fabs(basePwm) * (rightMmPerSec - leftMmPerSec) / speedSum;
    const float limit = kMotionConfig.wheelBalanceMaxPwm;
    return std::clamp(trim, -limit, limit);
}

float pidUpdate(PidState& state, const PidGains& gains, float error, float errorRate,
                float dtSec, float lower, float upper, bool integrate)
{
    if (!std::isfinite(error) || !std::isfinite(errorRate) ||
        !std::isfinite(dtSec) || dtSec <= 0)
        return 0;

    const float pd = gains.kp * error + gains.kd * errorRate;
    if (integrate && gains.ki != 0)
    {
        const float candidate = std::clamp(state.integral + error * dtSec,
                                           -gains.integralLimit, gains.integralLimit);
        const float output = pd + gains.ki * state.integral;
        const float change = gains.ki * (candidate - state.integral);
        const float boundedChange = std::clamp(change, std::min(0.0f, lower - output),
                                                        std::max(0.0f, upper - output));
        state.integral += boundedChange / gains.ki;
    }
    return std::clamp(pd + gains.ki * state.integral, lower, upper);
}

float distanceDemand(float remainingMm)
{
    if (remainingMm <= kMotionConfig.distanceToleranceMm)
        return 0;

    const float floor = kMotionConfig.straightMinPwm *
        approachFraction(remainingMm, kMotionConfig.distanceToleranceMm);
    return std::clamp(kMotionConfig.distanceKp * remainingMm, floor, kMotionConfig.straightBasePwm);
}

WheelResult monitorWheel(WheelMonitor& monitor, uint32_t nowUs, int previousPwm, int64_t delta)
{
    const int pwmDirection = direction(previousPwm);
    if (pwmDirection == 0)
    {
        monitor = {nowUs, 0};
        return WheelResult::Ok;
    }

    const int64_t projected = delta * pwmDirection;
    if (projected > 0)
        monitor.lastProgressUs = nowUs;
    monitor.wrongCounts = std::max(INT64_C(0), monitor.wrongCounts - projected);
    if (monitor.wrongCounts >= kMotionConfig.wrongDirectionCounts)
        return WheelResult::WrongDirection;
    if (nowUs - monitor.lastProgressUs >= kMotionConfig.stallTimeoutUs)
        return WheelResult::Stalled;
    return WheelResult::Ok;
}

float turnDemand(PidState& state, float errorDeg, float rateDegPerSec, float dtSec)
{
    if (std::fabs(errorDeg) <= kMotionConfig.turnToleranceDeg)
        return 0;

    const float output = pidUpdate(state, kMotionConfig.turnPid, errorDeg, -rateDegPerSec, dtSec,
                                  -kMotionConfig.turnMaxPwm, kMotionConfig.turnMaxPwm);
    const int sign = direction(errorDeg);
    if (output * sign <= 0)
        return 0; // Apply brake when the PD requests opposite torque.

    const float effort = std::fabs(output);
    const float taperEffort = kMotionConfig.turnPid.kp * kMotionConfig.turnToleranceDeg;
    const float effortFraction = std::min(effort / taperEffort, 1.0f);
    const float floor = kMotionConfig.turnMinPwm *
        std::min(approachFraction(std::fabs(errorDeg), kMotionConfig.turnToleranceDeg), effortFraction);
    return sign * std::max(effort, floor);
}

float slewPwm(float current, float target, float dtSec)
{
    const float rate = std::fabs(target) > std::fabs(current)
        ? kMotionConfig.risePwmPerSec : kMotionConfig.fallPwmPerSec;
    const float step = rate * dtSec;
    return current + std::clamp(target - current, -step, step);
}
