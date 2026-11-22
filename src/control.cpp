#include "control.h"

#include <algorithm>
#include <cmath>

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

float Pid::update(float error, float errorRate, float dtSec,
                  float lower, float upper, bool integrate)
{
    if (!std::isfinite(error) || !std::isfinite(errorRate) ||
        !std::isfinite(dtSec) || dtSec <= 0)
        return 0;

    const float pd = gains_.kp * error + gains_.kd * errorRate;
    if (integrate && gains_.ki != 0)
    {
        const float candidate = std::clamp(integral_ + error * dtSec,
                                           -gains_.integralLimit, gains_.integralLimit);
        const float change = gains_.ki * (candidate - integral_);
        const float output = pd + gains_.ki * candidate;
        if ((output <= upper || change < 0) && (output >= lower || change > 0))
            integral_ = candidate;
    }

    return std::clamp(pd + gains_.ki * integral_, lower, upper);
}

void HeadingController::reset()
{
    angleDeg_ = 0;
    rateDegPerSec_ = 0;
    setReference();
}

void HeadingController::update(float gyroDegPerSec, float dtSec, bool turning)
{
    const float signedRate = gyroDegPerSec * config_.gyroLeftSign * config_.gyroScale;
    rateDegPerSec_ += config_.gyroFilterAlpha * (signedRate - rateDegPerSec_);
    if (!turning && std::fabs(rateDegPerSec_) < config_.gyroDeadbandDegPerSec)
        rateDegPerSec_ = 0;

    // The measured rate avoids filter lag in turn angle. The filtered rate damps PWM.
    const float integrationRate = turning ? signedRate : rateDegPerSec_;
    if (std::fabs(integrationRate) >= config_.gyroDeadbandDegPerSec)
        angleDeg_ += integrationRate * dtSec;
}

float HeadingController::correction(float limitPwm, float dtSec)
{
    const float error = angleDeg();
    return pid_.update(error, rateDegPerSec_, dtSec, -limitPwm, limitPwm,
                       std::fabs(error) < config_.headingIntegralZoneDeg);
}

void DistanceController::start(float targetMm, EncoderCounts origin)
{
    targetMm_ = targetMm;
    origin_ = origin;
}

float DistanceController::travelMm(EncoderCounts counts) const
{
    return 0.5f * config_.mmPerCount() *
           (countDelta(counts.left, origin_.left) + countDelta(counts.right, origin_.right));
}

float DistanceController::remainingMm(float travelMm) const
{
    return (targetMm_ - travelMm) * direction(targetMm_);
}

bool DistanceController::brakeRequested(float travelMm) const
{
    return remainingMm(travelMm) <= config_.distanceToleranceMm;
}

bool DistanceController::atTarget(float travelMm) const
{
    return std::fabs(remainingMm(travelMm)) <= config_.distanceToleranceMm;
}

float DistanceController::demandPwm(float travelMm) const
{
    if (brakeRequested(travelMm))
        return 0;
    return direction(targetMm_) * std::clamp(config_.distanceKp * remainingMm(travelMm),
                                             config_.straightMinPwm, config_.straightBasePwm);
}

void WheelMonitor::reset(uint32_t nowUs)
{
    lastProgressUs_ = nowUs;
    wrongCounts_ = 0;
}

WheelMonitor::Result WheelMonitor::update(uint32_t nowUs, int previousPwm, int64_t delta,
                                         const MotionConfig& config)
{
    const int pwmDirection = direction(previousPwm);
    if (pwmDirection == 0)
    {
        reset(nowUs);
        return Result::Ok;
    }

    const int64_t projected = delta * pwmDirection;
    if (projected > 0)
        lastProgressUs_ = nowUs;
    wrongCounts_ = std::max(INT64_C(0), wrongCounts_ - projected);
    if (wrongCounts_ >= config.wrongDirectionCounts)
        return Result::WrongDirection;
    if (nowUs - lastProgressUs_ >= config.stallTimeoutUs)
        return Result::Stalled;
    return Result::Ok;
}

void TurnController::start(float targetDeg)
{
    targetDeg_ = targetDeg;
    pid_.reset();
}

bool TurnController::atTarget(float angleDeg) const
{
    return std::fabs(targetDeg_ - angleDeg) <= config_.turnToleranceDeg;
}

float TurnController::demandPwm(float angleDeg, float rateDegPerSec, float dtSec)
{
    const float error = targetDeg_ - angleDeg;
    if (atTarget(angleDeg))
        return 0;

    const float output = pid_.update(error, -rateDegPerSec, dtSec,
                                     -config_.turnMaxPwm, config_.turnMaxPwm);
    const int desiredDirection = direction(error);
    // Brake when angle and rate feedback ask for opposite torque.
    if (output * desiredDirection <= 0)
        return 0;
    return desiredDirection * std::max(std::fabs(output), config_.turnMinPwm);
}

float slewPwm(float current, float target, float dtSec, const MotionConfig& config)
{
    const float rate = std::fabs(target) > std::fabs(current)
        ? config.risePwmPerSec : config.fallPwmPerSec;
    const float step = rate * dtSec;
    return current + std::clamp(target - current, -step, step);
}
