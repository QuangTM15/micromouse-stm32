
#include "motion.h"

#include <algorithm>
#include <cmath>

#include "control.h"
#include "encoder.h"
#include "imu.h"
#include "motion_hardware.h"
#include "motor.h"

namespace
{

    // =====================================================
    // COMMAND TYPES
    // =====================================================

    enum class Command
    {
        None,
        Forward,
        Backward,
        Distance,
        Turn
    };

    // =====================================================
    // INTERNAL STATE
    // =====================================================

    MotionStatus status{};
    MotionEncoderData encoderData{};

    Command command = Command::None;

    PidState steeringPid{};
    WheelPwm pwm{};
    EncoderCounts previous{};

    uint32_t lastTickUs = 0;

    float target = 0.0f;
    float centerPwm = 0.0f;

    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    float headingOffsetDeg = 0.0f;

    // =====================================================
    // FINISH COMMAND
    // =====================================================

    void finish(MotionState nextState)
    {
        motors::disable();

        pwm = {};
        centerPwm = 0.0f;

        command = Command::None;
        status.state = nextState;
    }

    // =====================================================
    // HARDWARE FAULT
    // =====================================================

    void fail(MotionFault fault)
    {
        finish(MotionState::Error);
        status.fault = fault;
    }

    // =====================================================
    // START COMMAND
    // =====================================================

    bool start(Command next, float newTarget)
    {
        if (status.state != MotionState::Idle &&
            status.state != MotionState::Completed)
        {
            return false;
        }

        status = {};
        encoderData = {};

        command = next;

        status.state =
            next == Command::Turn
                ? MotionState::Turning
                : MotionState::Moving;

        target = newTarget;

        lastTickUs = hardware::nowUs();
        previous = encoders::read();

        steeringPid = {};
        pwm = {};

        centerPwm = 0.0f;
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;

        headingOffsetDeg = 0.0f;

        return true;
    }

    // =====================================================
    // STRAIGHT CONTROL
    // =====================================================

    void updateStraight(float dt)
    {
        // Forward = positive PWM.
        // Backward = negative PWM.
        float demand = kMotionConfig.straightBasePwm;

        if (command == Command::Backward)
        {
            demand = -kMotionConfig.straightBasePwm;
        }

        // Relative distance command.
        if (command == Command::Distance)
        {
            const int sign = direction(target);

            const float remainingMm =
                (target - status.travelMm) * sign;

            if (remainingMm <=
                kMotionConfig.distanceToleranceMm)
            {
                finish(MotionState::Completed);
                return;
            }

            demand = sign * distanceDemand(remainingMm);
        }

        // =================================================
        // ACCELERATION CONTROL
        // =================================================

        centerPwm = slewPwm(centerPwm, demand, dt);

        const float magnitude = std::fabs(centerPwm);

        const float limit = std::max(
            0.0f,
            std::min({magnitude * 0.5f,
                      static_cast<float>(
                          kMotionConfig.maxMotorPwm) -
                          magnitude,
                      kMotionConfig.headingCorrectionPwm}));

        // =================================================
        // HEADING PID
        // =================================================

        const float error =
            status.headingDeg - headingOffsetDeg;

        const float correction = pidUpdate(
            steeringPid,
            kMotionConfig.headingPid,
            error,
            status.rateDegPerSec,
            dt,
            -limit,
            limit,
            std::fabs(error) <
                kMotionConfig.headingIntegralZoneDeg);

        // =================================================
        // ENCODER WHEEL BALANCING
        // =================================================

        const float weight = std::max(
            0.0f,
            1.0f - std::max(
                       std::fabs(error) /
                           kMotionConfig.wheelBalanceFadeDeg,
                       std::fabs(status.rateDegPerSec) /
                           kMotionConfig.wheelBalanceFadeDegPerSec));

        const float trim = weight * wheelBalancePwm(
                                        leftSpeed,
                                        rightSpeed,
                                        centerPwm);

        const float steer = std::clamp(
            correction + trim,
            -limit,
            limit);

        // Same differential-drive steering convention
        // works with positive or negative center PWM.
        pwm = {
            static_cast<int>(
                std::lround(centerPwm + steer)),
            static_cast<int>(
                std::lround(centerPwm - steer))};

        motors::drive(pwm);
    }

    // =====================================================
    // TURN CONTROL
    // =====================================================

    void updateTurn(float dt)
    {
        const float remainingDeg =
            target - status.headingDeg;

        if (std::fabs(remainingDeg) <=
            kMotionConfig.turnToleranceDeg)
        {
            finish(MotionState::Completed);
            return;
        }

        const float demand = turnDemand(
            steeringPid,
            remainingDeg,
            status.rateDegPerSec,
            dt);

        centerPwm = slewPwm(centerPwm, demand, dt);

        const int effort =
            static_cast<int>(std::lround(centerPwm));

        pwm = {-effort, effort};

        motors::drive(pwm);
    }

    // =====================================================
    // UPDATE ACTIVE COMMAND
    // =====================================================

    void updateCommand()
    {
        const uint32_t tickUs = hardware::nowUs();

        const uint32_t elapsedUs =
            tickUs - lastTickUs;

        // Control frequency, not a command timeout.
        if (elapsedUs < kMotionConfig.periodUs)
        {
            return;
        }

        // =================================================
        // READ IMU
        // =================================================

        float gyroZ = 0.0f;

        const GyroRead result =
            imu::readGyroZ(gyroZ);

        if (result == GyroRead::Saturated)
        {
            fail(MotionFault::GyroSaturated);
            return;
        }

        if (result != GyroRead::Ok ||
            !std::isfinite(gyroZ) ||
            std::fabs(gyroZ) > 260.0f)
        {
            fail(MotionFault::ImuSample);
            return;
        }

        // =================================================
        // READ ENCODERS
        // =================================================

        const EncoderCounts counts = encoders::read();

        const int64_t dl =
            countDelta(counts.left, previous.left);

        const int64_t dr =
            countDelta(counts.right, previous.right);

        previous = counts;

        // Cap the integration step to prevent a long
        // scheduler pause from destabilizing the PID.
        const float dt = std::min(
            elapsedUs * 0.000001f,
            0.05f);

        lastTickUs = tickUs;

        // =================================================
        // DISTANCE
        // =================================================

        encoderData.leftCounts += dl;
        encoderData.rightCounts += dr;

        encoderData.leftMm =
            encoderData.leftCounts * kLeftMmPerCount;

        encoderData.rightMm =
            encoderData.rightCounts * kRightMmPerCount;

        status.travelMm = 0.5f * (encoderData.leftMm +
                                  encoderData.rightMm);

        // =================================================
        // WHEEL SPEED
        // =================================================

        leftSpeed +=
            kMotionConfig.wheelSpeedFilterAlpha *
            (dl * kLeftMmPerCount / dt -
             leftSpeed);

        rightSpeed +=
            kMotionConfig.wheelSpeedFilterAlpha *
            (dr * kRightMmPerCount / dt -
             rightSpeed);

        // =================================================
        // HEADING
        // =================================================

        const float signedRate =
            gyroZ *
            kMotionConfig.gyroLeftSign *
            kMotionConfig.gyroScale;

        status.rateDegPerSec +=
            kMotionConfig.gyroFilterAlpha *
            (signedRate -
             status.rateDegPerSec);

        const float rate =
            command == Command::Turn
                ? signedRate
                : status.rateDegPerSec;

        if (std::fabs(rate) >=
            kMotionConfig.gyroDeadbandDegPerSec)
        {
            status.headingDeg += rate * dt;
        }

        // =================================================
        // EXECUTE
        // =================================================

        if (command == Command::Turn)
        {
            updateTurn(dt);
        }
        else
        {
            updateStraight(dt);
        }
    }

} // namespace

// =====================================================
// INITIALIZATION
// =====================================================

bool motionInit()
{
    if (status.state == MotionState::Uninitialized)
    {
        const MotionFault fault =
            hardware::begin();

        if (fault != MotionFault::None)
        {
            fail(fault);
        }
        else
        {
            status.state = MotionState::Initializing;
        }
    }

    return !motionHasError();
}

// =====================================================
// CONTINUOUS FORWARD
// =====================================================

bool motionForward()
{
    return start(Command::Forward, 0.0f);
}

// =====================================================
// CONTINUOUS BACKWARD
// =====================================================

bool motionBackward()
{
    return start(Command::Backward, 0.0f);
}

// =====================================================
// RELATIVE DISTANCE
// =====================================================

bool motionMoveCounts(int32_t counts)
{
    if (counts == 0)
    {
        return false;
    }

    return start(
        Command::Distance,
        counts * kCenterMmPerCount);
}

// =====================================================
// RELATIVE TURN
// =====================================================

bool motionTurn(float angleDeg)
{
    if (!std::isfinite(angleDeg) ||
        angleDeg == 0.0f)
    {
        return false;
    }

    return start(Command::Turn, angleDeg);
}

// =====================================================
// NORMAL STOP
// =====================================================

void motionStop()
{
    if (status.state == MotionState::Moving ||
        status.state == MotionState::Turning ||
        status.state == MotionState::Stopping ||
        status.state == MotionState::Settling)
    {
        finish(MotionState::Idle);
    }
}

// =====================================================
// EMERGENCY STOP
// =====================================================

void motionEmergencyStop()
{
    if (motionHasError())
    {
        motors::disable();
    }
    else
    {
        fail(MotionFault::EmergencyStop);
    }
}

// =====================================================
// PUBLIC UPDATE
// =====================================================

void motionUpdate()
{
    if (status.state == MotionState::Initializing)
    {
        bool ready = false;

        const MotionFault fault =
            hardware::updateInit(ready);

        if (fault != MotionFault::None)
        {
            fail(fault);
        }
        else if (ready)
        {
            status.state = MotionState::Idle;
        }
    }
    else if (
        status.state == MotionState::Moving ||
        status.state == MotionState::Turning)
    {
        updateCommand();
    }

    hardware::serviceWatchdog();
}

// =====================================================
// STATUS
// =====================================================

MotionStatus motionGetStatus()
{
    return status;
}

// =====================================================
// ENCODER FEEDBACK
// =====================================================

MotionEncoderData motionGetEncoderData()
{
    MotionEncoderData data = encoderData;

    const EncoderCounts raw = encoders::read();

    data.leftRaw = raw.left;
    data.rightRaw = raw.right;

    return data;
}

// =====================================================
// MOTION STATE
// =====================================================

bool motionIsBusy()
{
    return status.state == MotionState::Initializing ||
           status.state == MotionState::Moving ||
           status.state == MotionState::Turning;
}

bool motionHasError()
{
    return status.state == MotionState::Error;
}

// =====================================================
// HEADING OFFSET
// =====================================================

bool motionSetHeadingOffset(float degrees)
{
    if (status.state != MotionState::Moving ||
        !std::isfinite(degrees) ||
        std::fabs(degrees) >
            kMotionConfig.maxHeadingOffsetDeg)
    {
        return false;
    }

    headingOffsetDeg = degrees;

    return true;
}
