
#pragma once

#include <Arduino.h>
#include "motion_config.h"

namespace board
{

    // =====================================================
    // MOTOR - DRV8833
    // =====================================================

    constexpr uint32_t motorSleep = PA0;

    constexpr uint32_t leftIn1 = PA1;
    constexpr uint32_t leftIn2 = PA2;

    constexpr uint32_t rightIn1 = PA3;
    constexpr uint32_t rightIn2 = PA4;

    // Physical motor direction
    constexpr bool reverseLeftMotor = true;
    constexpr bool reverseRightMotor = false;

    constexpr uint32_t motorPwmHz = 1000;

    // =====================================================
    // ENCODERS
    // =====================================================

    constexpr uint32_t leftEncoderA = PB5;
    constexpr uint32_t leftEncoderB = PB4;

    constexpr uint32_t rightEncoderA = PA8;
    constexpr uint32_t rightEncoderB = PA9;

    constexpr int leftEncoderSign = +1;
    constexpr int rightEncoderSign = -1;

    // =====================================================
    // I2C
    // =====================================================

    constexpr uint32_t scl = PB8;
    constexpr uint32_t sda = PB9;

    constexpr uint32_t i2cHz = 50000;
    constexpr uint32_t i2cTimeoutMs = 5;

    // =====================================================
    // IMU ICM20602
    // =====================================================

    constexpr uint8_t imuAddress = 0x69;

    constexpr uint32_t imuResetMs = 100;
    constexpr uint32_t imuStabilizeMs = 2000;

    constexpr uint16_t gyroCalibrationSamples = 500;
    constexpr uint32_t gyroCalibrationIntervalMs = 2;
    constexpr uint32_t gyroCalibrationTimeoutMs = 4000;

    constexpr float gyroCalibrationMaxSpanDegPerSec = 5.0f;
    constexpr float gyroCalibrationMaxBiasDegPerSec = 10.0f;

    // =====================================================
    // TOF SENSORS
    // =====================================================

    constexpr uint32_t unusedTofXshut1 = PB12;
    constexpr uint32_t unusedTofXshut2 = PB14;

    constexpr uint32_t rightTofXshut = PA10;
    constexpr uint32_t leftTofXshut = PB15;
    constexpr uint32_t frontTofXshut = PB13;

    constexpr uint8_t rightTofAddress = 0x31;
    constexpr uint8_t leftTofAddress = 0x32;
    constexpr uint8_t frontTofAddress = 0x33;

    constexpr uint32_t tofShutdownMs = 100;
    constexpr uint32_t tofWakeMs = 20;

    constexpr uint16_t tofInitTimeoutMs = 150;
    constexpr uint32_t tofBudgetUs = 33000;
    constexpr uint32_t tofReadTimeoutUs = 150000;

    // =====================================================
    // WATCHDOG
    // =====================================================

    constexpr uint32_t watchdogUs = 500000;

} // namespace board

static_assert(LED_BUILTIN == PC13);

static_assert(USER_BTN == board::motorSleep);

static_assert(
    kMotionConfig.periodUs <
        kMotionConfig.maxUpdateGapUs &&
    kMotionConfig.maxUpdateGapUs <
        board::watchdogUs);
