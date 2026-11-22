#include "motion.h"
#include "encoder.h"
#include "imu.h"
#include "motion_hardware.h"
#include "motor.h"
#include "physical_test.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>

namespace
{
enum class Output { Disabled, Drive, Brake };

// Each test runs in its own process, like booting the singleton on the MCU.
uint32_t clockUs = 0;
uint32_t sampleLatencyUs = 0;
unsigned samples = 0;
unsigned watchdogServices = 0;
bool initialized = true;
MotionFault beginFault = MotionFault::None;
MotionFault calibrationFault = MotionFault::None;
GyroRead gyroRead = GyroRead::Ok;
float gyroRate = 0;
EncoderCounts counts{};
WheelPwm pwm{};
Output output = Output::Disabled;

void near(float actual, float expected, float tolerance = 0.001f)
{
    assert(std::fabs(actual - expected) < tolerance);
}

void ready()
{
    assert(motionInit());
    assert(motionGetStatus().state == MotionState::Initializing && motionIsBusy());
    assert(!motionForward());
    motionUpdate();
    assert(motionGetStatus().state == MotionState::Idle && !motionIsBusy());
}

void tick(int left = 0, int right = 0, float rate = 0)
{
    counts.left += static_cast<uint32_t>(left);
    counts.right += static_cast<uint32_t>(right);
    gyroRate = rate;
    clockUs += kMotionConfig.periodUs;
    motionUpdate();
}

void settle()
{
    for (int i = 0; i < 25 && motionGetStatus().state == MotionState::Settling; ++i)
        tick();
}

void expectFault(MotionFault expected)
{
    assert(motionHasError() && !motionIsBusy());
    assert(motionGetStatus().state == MotionState::Error &&
           motionGetStatus().fault == expected && output == Output::Disabled);
    assert(!motionTurn(90) && !motionForward() && !motionInit());
    motionEmergencyStop();
    assert(motionGetStatus().fault == expected);
}

void testMath()
{
    PidState pid{};
    near(pidUpdate(pid, {1, 2, 3, 5}, 2, 1, 0.5f, -10, 10), 7);
    near(pidUpdate(pid, {1, 2, 3, 5}, 2, 1, 0, -10, 10), 0);
    near(pidUpdate(pid, {1, 2, 3, 5}, 2, 0, 1, -10, 10, false), 4);
    pid = {};
    near(pidUpdate(pid, {0, 1, 0, 10}, 8, 0, 1, -3, 3), 3);
    near(pidUpdate(pid, {0, 1, 0, 10}, -1, 0, 1, -3, 3), 2);

    near(slewPwm(0, 120, 0.02f), 24);
    near(slewPwm(120, 60, 0.02f), 70);
    near(distanceDemand(9), 31.5f);
    near(distanceDemand(8), 0);
    near(turnDemand(pid, 12, 100, 0.02f), 0);
    near(turnDemand(pid, -6, 0, 0.02f), -24);

    assert(countDelta(1, UINT32_MAX) == 2);
    assert(countDelta(UINT32_MAX, 1) == -2);
    assert(encoderEdgeDelta(false, +1) == +1 && encoderEdgeDelta(true, +1) == -1);
    assert(encoderEdgeDelta(false, -1) == -1 && encoderEdgeDelta(true, -1) == +1);
    near(wheelBalancePwm(69 * kLeftMmPerCount, 64 * kRightMmPerCount, 100), 0);
    assert(wheelBalancePwm(110, 100, 100) < 0);
    assert(wheelBalancePwm(-110, -100, -100) > 0);

    using namespace icm20602;
    near(toDegPerSec(decodeRawZ(0x00, 0x83)), 1);
    near(toDegPerSec(decodeRawZ(0xFF, 0x7D)), -1);
    assert(!atRail(decodeRawZ(0x7F, 0xF7)) && !atRail(decodeRawZ(0x80, 0x09)));
    assert(atRail(decodeRawZ(0x7F, 0xF8)) && atRail(decodeRawZ(0x80, 0x08)));
}

void testBridge()
{
    constexpr int limit = kMotionConfig.maxMotorPwm;
    for (const int command : {INT_MIN, -1000, -limit, -1, 0, 1, limit, 1000, INT_MAX})
    {
        const auto left = motorBridgePwm(command, true);
        const auto right = motorBridgePwm(command, false);
        assert(left.in1 == right.in2 && left.in2 == right.in1);
        assert(left.in1 <= limit && left.in2 <= limit && (left.in1 == 0 || left.in2 == 0));
        assert(static_cast<int>(right.in1) - right.in2 == std::clamp(command, -limit, limit));
    }
}

void testInit()
{
    assert(!motionForward() && !motionTurn(90) && !motionMoveCounts(40));
    initialized = false;
    assert(motionInit());
    motionUpdate();
    assert(motionIsBusy() && output == Output::Disabled && watchdogServices == 1);
    initialized = true;
    motionUpdate();
    assert(motionGetStatus().state == MotionState::Idle);
    assert(!motionMoveCounts(0) && !motionMoveCounts(1) && !motionMoveCounts(INT32_MIN));
    assert(!motionTurn(0) && !motionTurn(361) && !motionTurn(std::numeric_limits<float>::quiet_NaN()));
}

void testWatchdogReset()
{
    beginFault = MotionFault::WatchdogReset;
    assert(!motionInit());
    expectFault(MotionFault::WatchdogReset);
}

void testCalibrationFault()
{
    assert(motionInit());
    calibrationFault = MotionFault::ImuCalibration;
    motionUpdate();
    expectFault(MotionFault::ImuCalibration);
}

void testDistance(int sign)
{
    clockUs = UINT32_MAX - 10000;
    counts = sign > 0 ? EncoderCounts{UINT32_MAX - 2, UINT32_MAX - 2} : EncoderCounts{2, 2};
    ready();
    assert(motionMoveCounts(sign * 60) && !motionTurn(90));
    motionUpdate();
    assert(samples == 0 && output == Output::Disabled);
    tick();
    assert(direction(pwm.left) == sign && direction(pwm.right) == sign);
    for (int i = 0; i < 29; ++i)
        tick(sign * 2, sign * 2);
    assert(motionGetStatus().state == MotionState::Settling && output == Output::Brake);
    tick(sign, sign);
    settle();
    assert(motionGetStatus().state == MotionState::Completed && output == Output::Disabled);
    const auto data = motionGetEncoderData();
    assert(data.leftCounts == sign * 59 && data.rightCounts == sign * 59);
    near(data.leftMm, sign * 59 * kLeftMmPerCount);
    near(data.rightMm, sign * 59 * kRightMmPerCount);
    near(motionGetStatus().travelMm, 0.5f * (data.leftMm + data.rightMm));
    assert(data.leftRaw == counts.left);
    assert(motionTurn(45));
}

void testOvershoot()
{
    ready();
    assert(motionMoveCounts(60));
    tick();
    tick(70, 70);
    assert(output == Output::Brake);
    settle();
    expectFault(MotionFault::DistanceOutsideTolerance);
}

void testShortMoveStopsBothWheels()
{
    ready();
    assert(motionMoveCounts(12));
    tick();
    tick(15, 0);
    settle();
    expectFault(MotionFault::RightStall);
}

void testHeading()
{
    ready();
    assert(motionForward());
    tick();
    tick(1, 1, 40);
    assert(pwm.left > pwm.right);
    assert(motionSetHeadingOffset(10) && !motionSetHeadingOffset(20));
    tick(1, 1);
    assert(pwm.left < pwm.right);
    for (int i = 0; i < 7; ++i)
        tick(1, 1);
    assert(pwm.left >= pwm.right);

    motionStop();
    settle();
    while (motionGetStatus().state == MotionState::Stopping)
        tick();
    settle();

    assert(motionMoveCounts(-60));
    tick();
    tick(-1, -1, -40);
    assert(pwm.left < pwm.right && pwm.right < 0);
}

void testBalance()
{
    ready();
    assert(motionForward());
    tick();
    tick(2, 1);
    assert(pwm.left < pwm.right && pwm.left > 0);
}

void testTurn(float target)
{
    ready();
    assert(motionTurn(target));
    tick();
    const int sign = direction(target);
    assert(direction(pwm.left) == -sign && direction(pwm.right) == sign);
    // Lower the simulated rate near target to exercise the PD taper and settle.
    float simulatedYaw = 0;
    while (simulatedYaw < std::fabs(target))
    {
        assert(motionIsBusy());
        const float rate = std::min(100.0f, (std::fabs(target) - simulatedYaw) / 0.02f);
        tick(-sign, sign, sign * rate);
        simulatedYaw += rate * 0.02f;
    }
    assert(output == Output::Brake);
    settle();
    assert(motionGetStatus().state == MotionState::Completed && output == Output::Disabled);
    near(motionGetStatus().headingDeg, target, 0.01f);
}

void testTurnCorrection()
{
    ready();
    assert(motionTurn(45));
    tick();
    for (int i = 0; i < 28; ++i)
        tick(-1, 1, 100);
    assert(motionGetStatus().state == MotionState::Settling && output == Output::Brake);
    settle();
    assert(motionGetStatus().state == MotionState::Turning && output == Output::Brake);
    tick();
    assert(pwm.left > 0 && pwm.right < 0);
}

void testStop()
{
    ready();
    assert(motionForward());
    motionStop();
    assert(motionGetStatus().state == MotionState::Idle && output == Output::Disabled);
    assert(motionForward());
    tick();
    for (int i = 0; i < 6; ++i)
        tick(1, 1);
    const int initial = pwm.left;
    motionStop();
    assert(motionGetStatus().state == MotionState::Stopping && !motionTurn(45));
    tick(1, 1);
    assert(output == Output::Drive && pwm.left < initial && pwm.left > 0);
    tick();
    tick();
    assert(motionGetStatus().state == MotionState::Settling && output == Output::Brake);
    settle();
    assert(motionGetStatus().state == MotionState::Idle && output == Output::Disabled);
    assert(motionForward());
}

void testEmergency()
{
    ready();
    assert(motionForward());
    tick();
    assert(output == Output::Drive);
    motionEmergencyStop();
    expectFault(MotionFault::EmergencyStop);
    tick(1, 1);
    assert(output == Output::Disabled);
}

void testSensorFault(GyroRead result, float rate, MotionFault fault)
{
    ready();
    assert(motionForward());
    tick();
    gyroRead = result;
    tick(1, 1, rate);
    expectFault(fault);
}

void testWheelFault(bool left)
{
    ready();
    assert(motionForward());
    tick();
    for (int i = 0; i < 35; ++i)
        tick(left ? 0 : 1, left ? 1 : 0);
    expectFault(left ? MotionFault::LeftStall : MotionFault::RightStall);
}

void testWrongDirection()
{
    ready();
    assert(motionMoveCounts(60));
    tick();
    tick(1, -kMotionConfig.wrongDirectionCounts);
    expectFault(MotionFault::RightDirection);
}

void testTurnFault(float rate, MotionFault fault)
{
    ready();
    assert(motionTurn(90));
    tick();
    for (int i = 0; i < 35; ++i)
        tick(-1, 1, rate);
    expectFault(fault);
}

void testTiming(bool slowSample)
{
    ready();
    assert(motionForward());
    tick();
    if (slowSample)
        sampleLatencyUs = kMotionConfig.maxUpdateGapUs;
    else
        clockUs += kMotionConfig.maxUpdateGapUs;
    tick(1, 1);
    expectFault(MotionFault::UpdateGap);
}

void testTimeout()
{
    ready();
    assert(motionForward());
    while (motionIsBusy() && clockUs < kMotionConfig.continuousTimeoutUs)
        tick(1, 1);
    expectFault(MotionFault::Timeout);
}

void testPositioning()
{
    ready();
    assert(motionForward());
    tick();
    for (int i = 0; i < 30; ++i)
        tick(1, 1, 100);
    expectFault(MotionFault::HeadingError);
}

void testResetMenu()
{
    for (unsigned i = 0; i < kTestCount; ++i)
    {
        const auto select = saveTestSelection({i, TestStage::Select});
        assert(testAfterReset(select, true).index == (i + 1) % kTestCount);
        assert(testAfterReset(select, true).stage == TestStage::Guard);
        const auto offer = saveTestSelection({i, TestStage::Offer});
        const auto confirm = testAfterReset(offer, true);
        assert(confirm.index == i && confirm.stage == TestStage::Confirm);
        const auto confirmed = saveTestSelection(confirm);
        assert(testAfterReset(confirmed, true).stage == TestStage::ReadyWait);
        assert(testAfterReset(confirmed, false).stage == TestStage::Guard);
        assert(testAfterReset(confirmed ^ 1, true).stage == TestStage::Guard);
        assert(testAfterReset(saveTestSelection({i, TestStage::Guard}), true).stage == TestStage::Guard);
        assert(testAfterReset(saveTestSelection({i, TestStage::ReadyWait}), true).stage == TestStage::Fault);
        assert(testAfterReset(saveTestSelection({i, TestStage::Running}), true).stage == TestStage::Fault);
        const auto duringStop = saveTestSelection({i, TestStage::StopTestRunning});
        assert(testAfterReset(duringStop, true).stage == (i == 6 ? TestStage::Success : TestStage::Fault));
        for (unsigned step = 0; step < kPhysicalTest.plans[i].count; ++step)
        {
            const auto demand = kPhysicalTest.plans[i].steps[step];
            assert((demand.counts != 0) != (demand.turnDeg != 0));
            assert(demand.counts >= 0 && demand.counts <= kMotionConfig.maxMoveCounts);
        }
    }
}

struct Test { const char* name; void (*run)(); };
const Test cases[] = {
    {"math", testMath},
    {"bridge", testBridge},
    {"init", testInit},
    {"watchdog-reset", testWatchdogReset},
    {"calibration-fault", testCalibrationFault},
    {"forward-distance", [] { testDistance(1); }},
    {"reverse-distance", [] { testDistance(-1); }},
    {"overshoot", testOvershoot},
    {"short-move-wheel-fault", testShortMoveStopsBothWheels},
    {"heading", testHeading},
    {"balance", testBalance},
    {"turn-left-45", [] { testTurn(45); }},
    {"turn-right-45", [] { testTurn(-45); }},
    {"turn-left-90", [] { testTurn(90); }},
    {"turn-right-90", [] { testTurn(-90); }},
    {"turn-left-180", [] { testTurn(180); }},
    {"turn-right-180", [] { testTurn(-180); }},
    {"turn-full", [] { testTurn(360); }},
    {"turn-retry", testTurnCorrection},
    {"stop", testStop},
    {"emergency", testEmergency},
    {"imu-nack", [] { testSensorFault(GyroRead::Unavailable, 0, MotionFault::ImuSample); }},
    {"imu-stale", [] { testSensorFault(GyroRead::Pending, 0, MotionFault::ImuSample); }},
    {"imu-rail", [] { testSensorFault(GyroRead::Saturated, 0, MotionFault::GyroSaturated); }},
    {"imu-nonfinite", [] {
        testSensorFault(GyroRead::Ok, std::numeric_limits<float>::infinity(), MotionFault::ImuSample);
    }},
    {"left-stall", [] { testWheelFault(true); }},
    {"right-stall", [] { testWheelFault(false); }},
    {"encoder-sign", testWrongDirection},
    {"wrong-turn", [] { testTurnFault(-100, MotionFault::TurnDirection); }},
    {"turn-no-progress", [] { testTurnFault(0, MotionFault::TurnNoProgress); }},
    {"update-gap", [] { testTiming(false); }},
    {"slow-sample", [] { testTiming(true); }},
    {"timeout", testTimeout},
    {"heading-limit", testPositioning},
    {"reset-menu", testResetMenu},
};
}

MotionFault hardware::begin() { return beginFault; }
MotionFault hardware::updateInit(bool& isReady) { isReady = initialized; return calibrationFault; }
uint32_t hardware::nowUs() { return clockUs; }
void hardware::serviceWatchdog() { ++watchdogServices; }
EncoderCounts encoders::read() { return counts; }
GyroRead imu::readGyroZ(float& rate)
{
    ++samples;
    clockUs += sampleLatencyUs;
    rate = gyroRate;
    return gyroRead;
}
void motors::drive(WheelPwm demand)
{
    assert(std::abs(demand.left) <= kMotionConfig.maxMotorPwm &&
           std::abs(demand.right) <= kMotionConfig.maxMotorPwm);
    output = Output::Drive;
    pwm = demand;
}
void motors::brake() { output = Output::Brake; pwm = {}; }
void motors::disable() { output = Output::Disabled; pwm = {}; }

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--list") == 0)
    {
        for (const auto& test : cases)
            std::puts(test.name);
        return 0;
    }
    if (argc == 2)
    {
        for (const auto& test : cases)
        {
            if (std::strcmp(test.name, argv[1]) == 0)
            {
                test.run();
                std::printf("PASS %s\n", test.name);
                return 0;
            }
        }
    }
    return 2;
}
