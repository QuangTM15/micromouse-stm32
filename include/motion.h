
#pragma once

#include <stdint.h>

// =====================================================
// MOTION STATE
// =====================================================

enum class MotionState
{
    Uninitialized,
    Initializing,
    Idle,
    Moving,
    Turning,
    Stopping,
    Settling,
    Completed,
    Error
};

// =====================================================
// MOTION FAULT
// =====================================================

enum class MotionFault
{
    None,
    WatchdogReset,
    WatchdogInit,
    MotorSetup,
    EncoderSetup,
    I2cSetup,
    ImuInit,
    ImuCalibration,
    ImuSample,
    GyroSaturated,
    UpdateGap,
    Timeout,
    HeadingError,
    LeftStall,
    RightStall,
    LeftDirection,
    RightDirection,
    TurnDirection,
    TurnNoProgress,
    DistanceOutsideTolerance,
    EmergencyStop
};

// =====================================================
// STATUS
// =====================================================

struct MotionStatus
{
    MotionState state = MotionState::Uninitialized;
    MotionFault fault = MotionFault::None;

    float travelMm = 0.0f;
    float headingDeg = 0.0f;
    float rateDegPerSec = 0.0f;
};

struct MotionEncoderData
{
    uint32_t leftRaw = 0;
    uint32_t rightRaw = 0;

    int64_t leftCounts = 0;
    int64_t rightCounts = 0;

    float leftMm = 0.0f;
    float rightMm = 0.0f;
};

// =====================================================
// PUBLIC API
// =====================================================

bool motionInit(void);

// Continuous motion, no command timeout.
bool motionForward(void);
bool motionBackward(void);

// Relative target commands.
bool motionMoveCounts(int32_t counts);
bool motionTurn(float angleDeg);

// Motor stop.
void motionStop(void);
void motionEmergencyStop(void);

// Foreground PID update.
void motionUpdate(void);

// Feedback.
MotionStatus motionGetStatus(void);
MotionEncoderData motionGetEncoderData(void);

bool motionIsBusy(void);
bool motionHasError(void);

bool motionSetHeadingOffset(float degrees);
