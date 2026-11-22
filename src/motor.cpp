
#include "motor.h"
#include "config.h"

#include <algorithm>

// =====================================================
// HARDWARE MAPPING
// =====================================================
//
// DRV8833:
//
// LEFT:
//   IN1 = PA1 = TIM2_CH2
//   IN2 = PA2 = TIM2_CH3
//
// RIGHT:
//   IN3 = PA3 = TIM2_CH4
//   IN4 = PA4 = GPIO
//
// SLEEP = PA0
//
// =====================================================

namespace
{

    static_assert(
        board::motorSleep == PA0 &&
            board::leftIn1 == PA1 &&
            board::leftIn2 == PA2 &&
            board::rightIn1 == PA3 &&
            board::rightIn2 == PA4,
        "Motor pin configuration mismatch");

    bool ready = false;
    bool enabled = false;
    bool rightIn4High = false;

    // =====================================================
    // PWM CONVERSION
    // =====================================================

    uint32_t compareValue(uint16_t duty)
    {
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(TIM2->ARR + 1U) *
             duty) /
            255U);
    }

    // =====================================================
    // WRITE TIMER COMPARE VALUES
    // =====================================================

    void writePwm(
        uint16_t leftIn1,
        uint16_t leftIn2,
        uint16_t rightIn3)
    {
        // Update the three PWM channels.
        TIM2->CR1 |= TIM_CR1_UDIS;

        TIM2->CCR2 = compareValue(leftIn1);
        TIM2->CCR3 = compareValue(leftIn2);
        TIM2->CCR4 = compareValue(rightIn3);

        TIM2->CR1 &= ~TIM_CR1_UDIS;
    }

    // =====================================================
    // MOTOR ENABLE / DISABLE CONTROL
    // =====================================================

    void sleepBridge()
    {
        digitalWrite(board::motorSleep, LOW);
        enabled = false;
    }

    void wakeBridge()
    {
        // Apply compare values before enabling H-bridge.
        TIM2->EGR = TIM_EGR_UG;

        digitalWrite(board::motorSleep, HIGH);
        enabled = true;
    }

    // =====================================================
    // MOTOR INITIALIZATION
    // =====================================================

} // namespace

bool motors::begin()
{
    ready = false;
    enabled = false;
    rightIn4High = false;

    // Disable DRV8833 before configuration.
    pinMode(board::motorSleep, OUTPUT);
    digitalWrite(board::motorSleep, LOW);

    // PA4 is a GPIO output, not TIM2 PWM.
    pinMode(board::rightIn2, OUTPUT);
    digitalWrite(board::rightIn2, LOW);

    // Validate three PWM pins.
    const bool pinsValid =
        pinmap_peripheral(
            digitalPinToPinName(board::leftIn1),
            PinMap_TIM) == TIM2 &&

        pinmap_peripheral(
            digitalPinToPinName(board::leftIn2),
            PinMap_TIM) == TIM2 &&

        pinmap_peripheral(
            digitalPinToPinName(board::rightIn1),
            PinMap_TIM) == TIM2;

    if (!pinsValid)
    {
        return false;
    }

    // =============================================
    // RESET TIM2
    // =============================================

    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();

    // =============================================
    // TIMER CLOCK
    // =============================================

    const uint32_t apbHz =
        HAL_RCC_GetPCLK1Freq();

    const uint32_t timerHz =
        apbHz == HAL_RCC_GetHCLKFreq()
            ? apbHz
            : 2U * apbHz;

    TIM2->PSC = 0;

    TIM2->ARR =
        timerHz / board::motorPwmHz - 1U;

    // =============================================
    // PWM MODE 1 - CHANNEL 2
    // =============================================

    TIM2->CCMR1 =
        TIM_CCMR1_OC2PE |
        (6U << TIM_CCMR1_OC2M_Pos);

    // =============================================
    // PWM MODE 1 - CHANNEL 3 AND 4
    // =============================================

    TIM2->CCMR2 =
        TIM_CCMR2_OC3PE |
        TIM_CCMR2_OC4PE |
        (6U << TIM_CCMR2_OC3M_Pos) |
        (6U << TIM_CCMR2_OC4M_Pos);

    // =============================================
    // ENABLE PWM OUTPUTS
    // =============================================

    TIM2->CCER =
        TIM_CCER_CC2E |
        TIM_CCER_CC3E |
        TIM_CCER_CC4E;

    // Start with zero PWM.
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    TIM2->EGR = TIM_EGR_UG;

    TIM2->CR1 =
        TIM_CR1_ARPE |
        TIM_CR1_CEN;

    // =============================================
    // CONFIGURE TIMER AF PINS
    // =============================================

    pinmap_pinout(
        digitalPinToPinName(board::leftIn1),
        PinMap_TIM);

    pinmap_pinout(
        digitalPinToPinName(board::leftIn2),
        PinMap_TIM);

    pinmap_pinout(
        digitalPinToPinName(board::rightIn1),
        PinMap_TIM);

    ready = true;

    return true;
}

// =====================================================
// MOTOR DRIVE
// =====================================================

void motors::drive(WheelPwm request)
{
    if (!ready)
    {
        return;
    }

    // Both motors zero -> sleep driver.
    if (request.left == 0 &&
        request.right == 0)
    {
        disable();
        return;
    }

    // =============================================
    // LEFT MOTOR
    // =============================================

    // Left has two PWM-capable inputs.
    const BridgePwm left = motorBridgePwm(
        request.left,
        board::reverseLeftMotor);

    // =============================================
    // RIGHT MOTOR
    // =============================================

    int rightCommand = std::clamp(
        request.right,
        -kMotionConfig.maxMotorPwm,
        kMotionConfig.maxMotorPwm);

    if (board::reverseRightMotor)
    {
        rightCommand = -rightCommand;
    }

    const bool reverseRight =
        rightCommand < 0;

    const uint16_t rightDuty =
        static_cast<uint16_t>(
            std::abs(rightCommand));

    uint16_t rightIn3Duty = 0;

    if (rightCommand > 0)
    {
        // Forward:
        // IN3 = PWM
        // IN4 = LOW
        rightIn3Duty = rightDuty;
    }
    else if (rightCommand < 0)
    {
        // Reverse:
        // IN4 = HIGH.
        //
        // IN3 LOW  -> reverse drive
        // IN3 HIGH -> electrical brake
        //
        // Invert PWM so reverse drive occupies
        // rightDuty / 255 of each period.
        rightIn3Duty =
            static_cast<uint16_t>(
                255U - rightDuty);
    }
    else
    {
        // Coast right motor.
        rightIn3Duty = 0;
    }

    // =============================================
    // SAFE DIRECTION SWITCH
    // =============================================

    if (enabled &&
        reverseRight != rightIn4High)
    {
        // Briefly put the driver to sleep while
        // changing the direction GPIO.
        sleepBridge();
    }

    digitalWrite(
        board::rightIn2,
        reverseRight ? HIGH : LOW);

    rightIn4High = reverseRight;

    // =============================================
    // APPLY PWM
    // =============================================

    writePwm(
        left.in1,
        left.in2,
        rightIn3Duty);

    if (!enabled)
    {
        wakeBridge();
    }
}

// =====================================================
// ACTIVE BRAKE
// =====================================================

void motors::brake()
{
    if (!ready || !enabled)
    {
        return;
    }

    // Sleep during mode transition.
    sleepBridge();

    // DRV8833 brake: both inputs HIGH.
    digitalWrite(board::rightIn2, HIGH);
    rightIn4High = true;

    writePwm(255, 255, 255);

    wakeBridge();
}

// =====================================================
// DISABLE MOTORS
// =====================================================

void motors::disable()
{
    digitalWrite(board::motorSleep, LOW);

    enabled = false;

    // Reset right direction pin.
    digitalWrite(board::rightIn2, LOW);
    rightIn4High = false;

    if (ready)
    {
        writePwm(0, 0, 0);
    }
}
