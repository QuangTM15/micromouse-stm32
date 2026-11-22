#include "motion_hardware.h"

#include <Arduino.h>
#include <IWatchdog.h>
#include "config.h"
#include "encoder.h"
#include "i2c_bus.h"
#include "imu.h"
#include "motor.h"
#include "tof.h"

namespace
{
EncoderCounts calibrationOrigin{};
}

MotionFault hardware::begin()
{
    const bool watchdogReset = IWatchdog.isReset(false);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    const bool motorsReady = motors::begin();
    tof::shutdown();
    if (watchdogReset)
        return MotionFault::WatchdogReset;
    if (!motorsReady)
        return MotionFault::MotorSetup;

    IWatchdog.begin(board::watchdogUs);
    if (!IWatchdog.isEnabled())
        return MotionFault::WatchdogInit;
    if (!encoders::begin())
        return MotionFault::EncoderSetup;

    if (!i2c::begin())
        return MotionFault::I2cSetup;
    calibrationOrigin = encoders::read();
    return imu::begin() ? MotionFault::None : MotionFault::ImuInit;
}

MotionFault hardware::updateInit(bool& ready)
{
    const auto counts = encoders::read();
    if (counts.left != calibrationOrigin.left || counts.right != calibrationOrigin.right)
        return MotionFault::ImuCalibration;

    const auto result = imu::updateInit();
    ready = result == ImuInitResult::Ready;
    if (result == ImuInitResult::InitError)
        return MotionFault::ImuInit;
    if (result == ImuInitResult::CalibrationError)
        return MotionFault::ImuCalibration;
    return MotionFault::None;
}

uint32_t hardware::nowUs()
{
    return micros();
}

void hardware::serviceWatchdog()
{
    if (IWatchdog.isEnabled())
        IWatchdog.reload();
}
