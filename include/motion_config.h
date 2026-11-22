#pragma once

#include <stdint.h>

struct PidGains
{
    float kp;
    float ki;
    float kd;
    float integralLimit;
};

struct MotionConfig
{
    uint32_t periodUs = 20000;
    uint32_t maxUpdateGapUs = 60000;
    uint32_t distanceTimeoutUs = 5000000;
    uint32_t continuousTimeoutUs = 10000000;
    uint32_t turnTimeoutUs = 3500000;
    uint32_t longTurnTimeoutUs = 5000000;
    uint32_t stallTimeoutUs = 600000;
    uint32_t settleUs = 160000;
    uint32_t stopTimeoutUs = 1000000;

    float wheelDiameterMm = 34.0f;
    float leftCountsPerRev = 69.0f;
    float rightCountsPerRev = 64.0f;

    int gyroLeftSign = 1;
    float gyroScale = 1.0f;
    float gyroFilterAlpha = 0.35f;
    float gyroDeadbandDegPerSec = 1.0f;
    float stoppedDegPerSec = 5.0f;

    PidGains headingPid{2.0f, 0.06f, 0.15f, 25.0f};
    float headingCorrectionPwm = 20.0f;
    float headingIntegralZoneDeg = 15.0f;
    float maxStraightHeadingErrorDeg = 35.0f;
    float maxHeadingOffsetDeg = 10.0f;
    uint32_t headingOffsetLifetimeUs = 100000;

    float wheelBalanceMaxPwm = 5.0f;
    float wheelSpeedFilterAlpha = 0.2f;
    float wheelBalanceFadeDeg = 3.0f;
    float wheelBalanceFadeDegPerSec = 10.0f;

    float distanceKp = 3.5f; // PWM/mm
    float straightBasePwm = 120.0f;
    float straightMinPwm = 100.0f;
    float distanceToleranceMm = 8.0f;
    int32_t minMoveCounts = 12;
    int32_t maxMoveCounts = 220;

    PidGains turnPid{4.0f, 0.0f, 0.65f, 0.0f};
    float turnMinPwm = 90.0f;
    float turnMaxPwm = 120.0f;
    float turnToleranceDeg = 5.0f;
    float minTurnDeg = 10.0f;
    float maxTurnDeg = 360.0f;
    float wrongTurnDeg = 8.0f;
    float minTurnProgressDeg = 3.0f;

    int maxMotorPwm = 150;
    float risePwmPerSec = 1200.0f;
    float fallPwmPerSec = 2500.0f;
    int wrongDirectionCounts = 3;
};

constexpr MotionConfig kMotionConfig{};
constexpr float kWheelCircumferenceMm = 3.14159265f * kMotionConfig.wheelDiameterMm;
constexpr float kLeftMmPerCount = kWheelCircumferenceMm / kMotionConfig.leftCountsPerRev;
constexpr float kRightMmPerCount = kWheelCircumferenceMm / kMotionConfig.rightCountsPerRev;
constexpr float kCenterMmPerCount = kWheelCircumferenceMm /
                                    (0.5f * (kMotionConfig.leftCountsPerRev + kMotionConfig.rightCountsPerRev));

static_assert(kMotionConfig.maxMotorPwm <= 255 && kMotionConfig.maxMotorPwm > 0);
static_assert(kMotionConfig.straightMinPwm <= kMotionConfig.straightBasePwm &&
              kMotionConfig.straightBasePwm <= kMotionConfig.maxMotorPwm);
static_assert(kMotionConfig.turnMinPwm <= kMotionConfig.turnMaxPwm &&
              kMotionConfig.turnMaxPwm <= kMotionConfig.maxMotorPwm);
static_assert(kMotionConfig.minMoveCounts * kCenterMmPerCount > kMotionConfig.distanceToleranceMm);
static_assert(kMotionConfig.minTurnDeg > kMotionConfig.turnToleranceDeg);
