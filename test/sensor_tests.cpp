#include "config.h"
#include "i2c_bus.h"
#include "imu.h"
#include "tof.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
uint32_t now = 0;
uint8_t imuRegs[256]{};
int16_t rawGyro = 262;
bool sampleReady = true;
bool nack = false;

struct TofDevice
{
    uint32_t pin;
    uint8_t address = 0;
    bool irq = true;
    uint8_t rangeStatus = 11;
};

TofDevice sensors[] = {{board::rightTofXshut}, {board::leftTofXshut}, {board::frontTofXshut}};
unsigned spadMapWrites = 0;
unsigned timingWrites = 0;
unsigned continuousStarts = 0;
unsigned irqClears = 0;

bool tofRead(uint8_t address, uint8_t reg, uint8_t* data, size_t size)
{
    for (const auto& device : sensors)
    {
        if (device.address != address)
            continue;
        switch (reg)
        {
        case 0xC0:
            data[0] = 0xEE;
            break;
        case 0x92:
            data[0] = 3;
            break;
        case 0x83:
            data[0] = 1;
            break;
        case 0xB0:
            assert(size == 6);
            for (size_t i = 0; i < size; ++i)
                data[i] = 0xFF;
            break;
        case 0x13:
            data[0] = device.irq ? 4 : 0;
            break;
        case 0x14:
            assert(size == 12);
            data[0] = device.rangeStatus << 3U;
            data[10] = 1;
            data[11] = 44;
            break;
        default:
            data[0] = 0;
            break;
        }
        return true;
    }
    return false;
}

void testImu()
{
    imuRegs[0x75] = 0;
    assert(!imu::begin());
    imuRegs[0x75] = 0x12;
    assert(imu::begin() && imuRegs[0x6B] == 0x80);
    assert(imu::updateInit() == ImuInitResult::Pending);
    now += board::imuResetMs;
    assert(imu::updateInit() == ImuInitResult::Pending && imuRegs[0x6B] == 1);
    now += board::imuStabilizeMs;
    assert(imu::updateInit() == ImuInitResult::Pending);
    for (unsigned sample = 1; sample <= board::gyroCalibrationSamples; ++sample)
    {
        now += board::gyroCalibrationIntervalMs;
        const auto expected = sample == board::gyroCalibrationSamples
            ? ImuInitResult::Ready : ImuInitResult::Pending;
        assert(imu::updateInit() == expected);
    }

    float rate = 0;
    rawGyro = -131;
    assert(imu::readGyroZ(rate) == GyroRead::Ok && std::fabs(rate - (-3)) < 0.001f);
    sampleReady = false;
    assert(imu::readGyroZ(rate) == GyroRead::Pending);
    sampleReady = true;
    rawGyro = INT16_MAX;
    assert(imu::readGyroZ(rate) == GyroRead::Saturated);
    nack = true;
    assert(imu::readGyroZ(rate) == GyroRead::Unavailable);
    nack = false;
}

void testCalibrationRejectsMotion()
{
    rawGyro = 0;
    assert(imu::begin());
    now += board::imuResetMs;
    assert(imu::updateInit() == ImuInitResult::Pending);
    now += board::imuStabilizeMs;
    assert(imu::updateInit() == ImuInitResult::Pending);
    now += board::gyroCalibrationIntervalMs;
    assert(imu::updateInit() == ImuInitResult::Pending);
    rawGyro = 6 * 131;
    now += board::gyroCalibrationIntervalMs;
    assert(imu::updateInit() == ImuInitResult::CalibrationError);
}

void testTof()
{
    tof::begin();
    uint16_t range = 0;
    assert(tof::poll(TofPosition::Right, range) == TofResult::Unavailable);
    auto result = TofInitResult::Pending;
    for (unsigned i = 0; i < 250 && result == TofInitResult::Pending; ++i)
    {
        ++now;
        result = tof::updateInit();
    }
    assert(result == TofInitResult::Ready);
    assert(sensors[0].address == 0x31 && sensors[1].address == 0x32 && sensors[2].address == 0x33);
    assert(spadMapWrites == 3 && timingWrites == 3 && continuousStarts == 3);

    assert(tof::poll(TofPosition::Right, range) == TofResult::Ready && range == 300);
    assert(irqClears > 0);
    sensors[0].irq = false;
    assert(tof::poll(TofPosition::Right, range) == TofResult::Pending);
    now += board::tofReadTimeoutUs / 1000;
    assert(tof::poll(TofPosition::Right, range) == TofResult::Timeout);
    sensors[1].rangeStatus = 4;
    assert(tof::poll(TofPosition::Left, range) == TofResult::BadRange && range == 300);
    nack = true;
    assert(tof::poll(TofPosition::Front, range) == TofResult::BusError);

    tof::begin();
    now += board::tofShutdownMs;
    assert(tof::updateInit() == TofInitResult::Pending);
    now += board::tofWakeMs;
    assert(tof::updateInit() == TofInitResult::Failed);
    assert(tof::poll(TofPosition::Right, range) == TofResult::Unavailable);
    for (const auto& device : sensors)
        assert(device.address == 0);
}
}

uint32_t millis() { return now; }
uint32_t micros() { return now * 1000; }
void pinMode(uint32_t, PinMode) {}
void digitalWrite(uint32_t pin, PinStatus status)
{
    for (auto& device : sensors)
    {
        if (device.pin == pin)
            device.address = status == HIGH ? 0x29 : 0;
    }
}

bool i2c::read(uint8_t address, uint8_t reg, uint8_t* data, size_t size)
{
    if (nack)
        return false;
    if (address != board::imuAddress)
        return tofRead(address, reg, data, size);
    if (reg == 0x3A)
    {
        assert(size == 15);
        data[0] = sampleReady ? 1 : 0;
        data[13] = static_cast<uint16_t>(rawGyro) >> 8U;
        data[14] = static_cast<uint8_t>(rawGyro);
    }
    else
    {
        assert(size == 1);
        data[0] = imuRegs[reg];
    }
    return true;
}

bool i2c::write(uint8_t address, uint8_t reg, const uint8_t* data, size_t size)
{
    if (nack)
        return false;
    if (address == board::imuAddress)
    {
        assert(size == 1);
        imuRegs[reg] = data[0];
        return true;
    }
    for (auto& device : sensors)
    {
        if (device.address != address)
            continue;
        if (reg == 0x8A)
            device.address = data[0];
        if (reg == 0xB0 && size == 6)
        {
            assert(data[0] == 7 && data[1] == 0 && data[5] == 0);
            ++spadMapWrites;
        }
        if (reg == 0x71 && size == 2)
            ++timingWrites;
        if (address != 0x29 && reg == 0 && data[0] == 2)
            ++continuousStarts;
        if (address != 0x29 && reg == 0x0B && data[0] == 1)
            ++irqClears;
        return true;
    }
    return false;
}

bool i2c::write(uint8_t address, uint8_t reg, uint8_t value)
{
    return write(address, reg, &value, 1);
}

int main()
{
    testImu();
    testCalibrationRejectsMotion();
    testTof();
    std::puts("PASS IMU initialization and calibration");
    std::puts("PASS IMU rejects movement");
    std::puts("PASS ToF addressing, ranging and faults");
}
