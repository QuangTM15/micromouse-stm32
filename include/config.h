#pragma once

#include <Arduino.h>
#include "motion_config.h"

namespace board
{
constexpr uint32_t motorSleep = PA0;
constexpr uint32_t leftIn1 = PA1;
constexpr uint32_t leftIn2 = PA2;
constexpr uint32_t rightIn1 = PA3;
constexpr uint32_t rightIn2 = PA5;
constexpr bool reverseLeftMotor = true;
constexpr bool reverseRightMotor = false;

constexpr uint32_t leftEncoderA = PB5;
constexpr uint32_t leftEncoderB = PB4;
constexpr uint32_t rightEncoderA = PA8;
constexpr uint32_t rightEncoderB = PA9;
// At A rising, B low is +1 before applying these unverified phase signs.
constexpr int leftEncoderSign = +1;
constexpr int rightEncoderSign = +1;

constexpr uint32_t scl = PB8;
constexpr uint32_t sda = PB9;
constexpr uint32_t i2cHz = 50000;
constexpr uint8_t imuAddress = 0x69;
constexpr uint32_t imuWakeMs = 50;
constexpr uint16_t gyroCalibrationSamples = 500;
constexpr uint32_t gyroCalibrationIntervalMs = 2;
constexpr float gyroCalibrationMaxSpanDegPerSec = 5.0f;
constexpr float gyroCalibrationMaxBiasDegPerSec = 10.0f;

constexpr uint32_t unusedTofXshut1 = PB12; // U3
constexpr uint32_t unusedTofXshut2 = PB14; // U2
constexpr uint32_t rightTofXshut = PA10;   // U4
constexpr uint32_t leftTofXshut = PB15;    // U5
constexpr uint32_t frontTofXshut = PB13;   // U6
constexpr uint8_t rightTofAddress = 0x31;
constexpr uint8_t leftTofAddress = 0x32;
constexpr uint8_t frontTofAddress = 0x33;
constexpr uint32_t tofShutdownMs = 100;
constexpr uint32_t tofWakeMs = 20;
constexpr uint16_t tofInitTimeoutMs = 150;
constexpr uint32_t tofBudgetUs = 33000;
constexpr uint32_t tofReadTimeoutUs = 150000;

constexpr uint32_t watchdogUs = 500000;
}

enum class Phase1Test { Passive, StraightShort, TurnLeft90, TurnRight90, Turn180,
                        StraightCell, FullSequence, TurnRight180 };

#ifndef PHASE1_TEST
#define PHASE1_TEST 1
#endif
#ifndef PHASE1_ENABLE_TOF
#define PHASE1_ENABLE_TOF 1
#endif
#ifndef PHASE1_DEBUG
#define PHASE1_DEBUG 0
#endif

constexpr auto kPhase1Test = static_cast<Phase1Test>(PHASE1_TEST);
constexpr uint32_t kTestStartupWaitMs = 2000;
constexpr uint32_t kTestPauseMs = 1000;
constexpr uint32_t kDebugSampleMs = 200;
constexpr float kShortTestMm = 60.0f;
constexpr float kCellTestMm = 180.0f;

static_assert(PHASE1_TEST >= 0 && PHASE1_TEST <= 7, "Invalid Phase 1 test");
static_assert(board::leftEncoderSign == 1 || board::leftEncoderSign == -1, "Invalid left encoder sign");
static_assert(board::rightEncoderSign == 1 || board::rightEncoderSign == -1, "Invalid right encoder sign");
static_assert(kMotionConfig.gyroLeftSign == 1 || kMotionConfig.gyroLeftSign == -1, "Invalid gyro sign");
static_assert(kMotionConfig.periodUs > 0 && kMotionConfig.periodUs < kMotionConfig.maxUpdateGapUs &&
              kMotionConfig.maxUpdateGapUs < board::watchdogUs, "Invalid control timing");
static_assert(kMotionConfig.maxMotorPwm > 0 && kMotionConfig.maxMotorPwm <= 255 &&
              kMotionConfig.straightMinPwm >= 1 && kMotionConfig.straightMinPwm <= kMotionConfig.straightBasePwm &&
              kMotionConfig.straightBasePwm <= kMotionConfig.maxMotorPwm &&
              kMotionConfig.turnMinPwm >= 1 && kMotionConfig.turnMinPwm <= kMotionConfig.turnMaxPwm &&
              kMotionConfig.turnMaxPwm <= kMotionConfig.maxMotorPwm, "Invalid PWM limits");
static_assert(kMotionConfig.wheelDiameterMm > 0 && kMotionConfig.encoderEdgesPerMotorRev > 0 &&
              kMotionConfig.gearRatio > 0, "Invalid encoder scale");
