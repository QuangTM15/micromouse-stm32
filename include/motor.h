#pragma once

#include <algorithm>
#include "control.h"

struct BridgePwm
{
    uint16_t in1 = 0;
    uint16_t in2 = 0;
};

constexpr BridgePwm motorBridgePwm(int pwm, bool reverse)
{
    const int limit = kMotionConfig.maxMotorPwm;
    const int bounded = std::clamp(pwm, -limit, limit);
    const int electrical = reverse ? -bounded : bounded;
    return {static_cast<uint16_t>(std::max(0, electrical)),
            static_cast<uint16_t>(std::max(0, -electrical))};
}

namespace motors
{
    bool begin();
    void drive(WheelPwm pwm);
    void brake();
    void disable();
}
