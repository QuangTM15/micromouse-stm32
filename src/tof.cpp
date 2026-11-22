#include "tof.h"
#include "config.h"
#include "i2c_bus.h"

// Initialization sequences adapted from Pololu VL53L0X 1.3.1 and STSW-IMG005.
// See VL53L0X-LICENSE.txt for the notices and redistribution terms.
namespace
{
constexpr uint8_t kDefaultAddress = 0x29;
constexpr uint8_t kStart = 0x00;
constexpr uint8_t kSequence = 0x01;
constexpr uint8_t kInterruptClear = 0x0B;
constexpr uint8_t kInterruptStatus = 0x13;
constexpr uint8_t kRangeStatus = 0x14;
constexpr uint8_t kAddressReg = 0x8A;
constexpr uint8_t kModelId = 0xC0;
constexpr uint8_t kModelValue = 0xEE;
constexpr uint8_t kRangeComplete = 11;
constexpr uint8_t kRangeReadyMask = 0x07;
constexpr uint8_t kRangingSequence = 0xE8;

struct RegisterValue { uint8_t reg; uint8_t value; };
constexpr RegisterValue kTuning[] = {
{0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00},
    {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01},
    {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09},
    {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
    {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96},
    {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
    {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14},
    {0x49, 0xFF}, {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
    {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF},
    {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06}, {0x20, 0x1A},
    {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01},
    {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00},
    {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
    {0xFF, 0x01}, {0x0D, 0x01}, {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
    {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

struct Channel
{
    uint32_t xshut;
    uint8_t address;
    uint32_t lastResultUs;
};

Channel channels[] = {
    {board::rightTofXshut, board::rightTofAddress, 0},
    {board::leftTofXshut, board::leftTofAddress, 0},
    {board::frontTofXshut, board::frontTofAddress, 0}
};
constexpr unsigned kChannelCount = sizeof(channels) / sizeof(channels[0]);

enum class Phase { Off, ShutdownWait, WakeWait, SpadWait, VhvWait, PhaseWait, Ready, Failed };
Phase phase = Phase::Off;
unsigned channelIndex = 0;
uint32_t phaseAtMs = 0;
uint8_t stopVariable = 0;

void holdShutdown(uint32_t pin)
{
    pinMode(pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(pin, LOW);
}

void enterPhase(Phase next)
{
    phase = next;
    phaseAtMs = millis();
}

bool readRegister(uint8_t reg, uint8_t& value)
{
    return i2c::read(kDefaultAddress, reg, &value, 1);
}

bool writeRegister(uint8_t reg, uint8_t value)
{
    return i2c::write(kDefaultAddress, reg, value);
}

bool modifyRegister(uint8_t reg, uint8_t clear, uint8_t set)
{
    uint8_t value = 0;
    return readRegister(reg, value) && writeRegister(reg, (value & ~clear) | set);
}

bool writeSequence(const RegisterValue* sequence, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (!writeRegister(sequence[i].reg, sequence[i].value))
            return false;
    }
    return true;
}

template <size_t N>
bool writeSequence(const RegisterValue (&sequence)[N])
{
    return writeSequence(sequence, N);
}

bool beginSpadRead()
{
    const RegisterValue saveStop[] = {{0x88, 0}, {0x80, 1}, {0xFF, 1}, {0x00, 0}};
    const RegisterValue requestSpads[] = {
        {0xFF, 7}, {0x81, 1}, {0x80, 1}, {0x94, 0x6B}, {0x83, 0}
    };
    const uint8_t signalLimit[] = {0, 32}; // 0.25 MCPS in 9.7 fixed point.
    uint8_t model = 0;
    return readRegister(kModelId, model) && model == kModelValue &&
        modifyRegister(0x89, 0, 1) && writeSequence(saveStop) && readRegister(0x91, stopVariable) &&
        writeSequence({{0x00, 1}, {0xFF, 0}, {0x80, 0}}) &&
        modifyRegister(0x60, 0, 0x12) &&
        i2c::write(kDefaultAddress, 0x44, signalLimit, sizeof(signalLimit)) &&
        writeSequence({{kSequence, 0xFF}, {0x80, 1}, {0xFF, 1}, {0x00, 0}, {0xFF, 6}}) &&
        modifyRegister(0x83, 0, 4) && writeSequence(requestSpads);
}

bool configureSpads()
{
    constexpr uint8_t kSpadMap = 0xB0;
    uint8_t info = 0;
    uint8_t map[6]{};
    if (!writeRegister(0x83, 1) || !readRegister(0x92, info) ||
        !writeSequence({{0x81, 0}, {0xFF, 6}}) || !modifyRegister(0x83, 4, 0) ||
        !writeSequence({{0xFF, 1}, {0x00, 1}, {0xFF, 0}, {0x80, 0}}) ||
        !i2c::read(kDefaultAddress, kSpadMap, map, sizeof(map)) ||
        !writeSequence({{0xFF, 1}, {0x4F, 0}, {0x4E, 0x2C}, {0xFF, 0}, {0xB6, 0xB4}}))
        return false;

    const unsigned required = info & 0x7F;
    const unsigned first = (info & 0x80) ? 12 : 0;
    unsigned enabled = 0;
    for (unsigned i = 0; i < 48; ++i)
    {
        const uint8_t mask = 1U << (i % 8);
        if (i < first || enabled == required)
            map[i / 8] &= ~mask;
        else if ((map[i / 8] & mask) != 0)
            ++enabled;
    }
    return required > 0 && enabled == required &&
        i2c::write(kDefaultAddress, kSpadMap, map, sizeof(map));
}

constexpr uint32_t macroPeriodNs(uint32_t periodPclks)
{
    return (2304U * periodPclks * 1655U + 500U) / 1000U;
}

bool setTimingBudget()
{
    // Fixed sequence 0xE8 and periods/timeouts from kTuning. Subtract DSS,
    // pre-range and fixed overhead, then encode the final-range timeout.
    constexpr auto preMacroNs = macroPeriodNs(14);
    constexpr auto finalMacroNs = macroPeriodNs(10);
    constexpr uint32_t preMclks = 0x96 + 1;
    constexpr uint32_t dssUs = ((0x25 + 1) * preMacroNs + 500) / 1000;
    constexpr uint32_t preUs = (preMclks * preMacroNs + 500) / 1000;
    constexpr uint32_t usedUs = 1910 + 960 + 2 * (dssUs + 690) + preUs + 660 + 550;
    static_assert(board::tofBudgetUs > usedUs);
    const uint32_t finalUs = board::tofBudgetUs - usedUs;
    uint32_t mantissa = (finalUs * 1000 + finalMacroNs / 2) / finalMacroNs + preMclks - 1;
    uint8_t exponent = 0;
    while (mantissa > 255)
    {
        mantissa >>= 1;
        ++exponent;
    }
    const uint8_t timeout[] = {exponent, static_cast<uint8_t>(mantissa)};
    return i2c::write(kDefaultAddress, 0x71, timeout, sizeof(timeout));
}

bool beginReferenceCalibration()
{
    return configureSpads() && writeSequence(kTuning) && writeRegister(0x0A, 4) &&
        modifyRegister(0x84, 0x10, 0) && writeRegister(kInterruptClear, 1) &&
        writeRegister(kSequence, kRangingSequence) && setTimingBudget() &&
        writeRegister(kSequence, 1) && writeRegister(kStart, 0x41);
}

bool finishReferenceCalibration()
{
    return writeRegister(kInterruptClear, 1) && writeRegister(kStart, 0);
}

bool startRanging(Channel& channel)
{
    uint8_t model = 0;
    const RegisterValue restoreStop[] = {
        {0x80, 1}, {0xFF, 1}, {0x00, 0}, {0x91, stopVariable},
        {0x00, 1}, {0xFF, 0}, {0x80, 0}
    };
    if (!finishReferenceCalibration() || !writeRegister(kSequence, kRangingSequence) ||
        !writeSequence(restoreStop) || !writeRegister(kAddressReg, channel.address) ||
        !i2c::read(channel.address, kModelId, &model, 1) || model != kModelValue ||
        !i2c::write(channel.address, kStart, 2))
        return false;
    channel.lastResultUs = micros();
    return true;
}

void wakeChannel()
{
    digitalWrite(channels[channelIndex].xshut, HIGH); // Release to the carrier's pull-up.
    enterPhase(Phase::WakeWait);
}

TofInitResult fail()
{
    tof::shutdown();
    phase = Phase::Failed;
    return TofInitResult::Failed;
}
}

void tof::shutdown()
{
    holdShutdown(board::unusedTofXshut1);
    holdShutdown(board::unusedTofXshut2);
    for (const auto& channel : channels)
        holdShutdown(channel.xshut);
    phase = Phase::Off;
}

void tof::begin()
{
    shutdown();
    channelIndex = 0;
    enterPhase(Phase::ShutdownWait);
}

TofInitResult tof::updateInit()
{
    const uint32_t elapsedMs = millis() - phaseAtMs;
    switch (phase)
    {
    case Phase::Off:
        return TofInitResult::Off;
    case Phase::Ready:
        return TofInitResult::Ready;
    case Phase::Failed:
        return TofInitResult::Failed;
    case Phase::ShutdownWait:
        if (elapsedMs >= board::tofShutdownMs)
            wakeChannel();
        return TofInitResult::Pending;
    case Phase::WakeWait:
        if (elapsedMs < board::tofWakeMs)
            return TofInitResult::Pending;
        if (!beginSpadRead())
            return fail();
        enterPhase(Phase::SpadWait);
        return TofInitResult::Pending;
    default:
        break;
    }

    uint8_t value = 0;
    const bool spad = phase == Phase::SpadWait;
    if (elapsedMs >= board::tofInitTimeoutMs || !readRegister(spad ? 0x83 : kInterruptStatus, value))
        return fail();
    if ((value & (spad ? 0xFF : kRangeReadyMask)) == 0)
        return TofInitResult::Pending;

    if (spad)
    {
        if (!beginReferenceCalibration())
            return fail();
        enterPhase(Phase::VhvWait);
    }
    else if (phase == Phase::VhvWait)
    {
        if (!finishReferenceCalibration() || !writeRegister(kSequence, 2) || !writeRegister(kStart, 1))
            return fail();
        enterPhase(Phase::PhaseWait);
    }
    else
    {
        if (!startRanging(channels[channelIndex]))
            return fail();
        if (++channelIndex == kChannelCount)
        {
            enterPhase(Phase::Ready);
            return TofInitResult::Ready;
        }
        wakeChannel();
    }
    return TofInitResult::Pending;
}

TofResult tof::poll(TofPosition position, uint16_t& millimeters)
{
    const auto index = static_cast<unsigned>(position);
    if (phase != Phase::Ready || index >= kChannelCount)
        return TofResult::Unavailable;

    auto& channel = channels[index];
    uint8_t irq = 0;
    if (!i2c::read(channel.address, kInterruptStatus, &irq, 1))
        return TofResult::BusError;
    if ((irq & kRangeReadyMask) == 0)
        return micros() - channel.lastResultUs >= board::tofReadTimeoutUs
            ? TofResult::Timeout : TofResult::Pending;

    uint8_t data[12]{};
    if (!i2c::read(channel.address, kRangeStatus, data, sizeof(data)) ||
        !i2c::write(channel.address, kInterruptClear, 1))
        return TofResult::BusError;
    channel.lastResultUs = micros();

    const uint16_t range = (static_cast<uint16_t>(data[10]) << 8U) | data[11];
    if (((data[0] >> 3U) & 0x0F) != kRangeComplete || range == 0 || range == UINT16_MAX)
        return TofResult::BadRange;
    millimeters = range;
    return TofResult::Ready;
}
