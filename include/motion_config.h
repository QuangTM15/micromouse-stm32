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
    uint32_t distanceTimeoutUs = 2500000;
    uint32_t turnTimeoutUs = 3500000;
    uint32_t halfTurnTimeoutUs = 5000000;
    uint32_t stallTimeoutUs = 600000;
    uint32_t settleUs = 160000;

    // Inherited scale for A rising edges, pending measurement on the assembled robot.
    float wheelDiameterMm = 34.0f;
    float encoderEdgesPerMotorRev = 12.0f;
    float gearRatio = 4.0f;

    int gyroLeftSign = 1;
    float gyroScale = 1.0f;
    float gyroFilterAlpha = 0.35f;
    float gyroDeadbandDegPerSec = 1.0f;
    float stoppedDegPerSec = 5.0f;

    PidGains headingPid{2.0f, 0.06f, 0.15f, 25.0f};
    float headingCorrectionPwm = 20.0f;
    float headingIntegralZoneDeg = 15.0f;
    float maxStraightHeadingErrorDeg = 35.0f;

    float distanceKp = 3.5f;       // PWM/mm
    float straightBasePwm = 120.0f;
    float straightMinPwm = 100.0f;
    float distanceToleranceMm = 8.0f;
    float minDistanceMm = 20.0f;
    float maxDistanceMm = 180.0f;

    PidGains turnPid{4.0f, 0.0f, 0.65f, 0.0f};
    float turnMinPwm = 90.0f;
    float turnMaxPwm = 120.0f;
    float turnToleranceDeg = 5.0f;
    float minTurnDeg = 10.0f;
    float maxTurnDeg = 180.0f;
    float wrongTurnDeg = 8.0f;
    float minTurnProgressDeg = 3.0f;

    int maxMotorPwm = 150;
    float risePwmPerSec = 1200.0f;
    float fallPwmPerSec = 2500.0f;
    int wrongDirectionCounts = 3;

    float mmPerCount() const
    {
        return 3.14159265f * wheelDiameterMm / (encoderEdgesPerMotorRev * gearRatio);
    }
};

constexpr MotionConfig kMotionConfig{};
