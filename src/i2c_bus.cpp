
#include <Arduino.h>
#include <Wire.h>

#include "i2c_bus.h"
#include "config.h"

namespace
{
    TwoWire bus(board::sda, board::scl);
    bool ready = false;
}

bool i2c::begin()
{
    bus.begin();
    bus.setClock(board::i2cHz);

    ready = true;
    return ready;
}

bool i2c::read(uint8_t address,
               uint8_t reg,
               uint8_t *data,
               size_t length)
{
    if (!ready || address > 0x7F ||
        data == nullptr || length == 0)
    {
        return false;
    }

    bus.beginTransmission(address);
    bus.write(reg);

    if (bus.endTransmission(false) != 0)
    {
        return false;
    }

    size_t received = bus.requestFrom(
        address, length, true);

    if (received != length)
    {
        return false;
    }

    for (size_t i = 0; i < length; ++i)
    {
        data[i] = bus.read();
    }

    return true;
}

bool i2c::write(uint8_t address,
                uint8_t reg,
                const uint8_t *data,
                size_t length)
{
    if (!ready || address > 0x7F ||
        data == nullptr || length == 0)
    {
        return false;
    }

    bus.beginTransmission(address);
    bus.write(reg);
    bus.write(data, length);

    return bus.endTransmission(true) == 0;
}

bool i2c::write(uint8_t address,
                uint8_t reg,
                uint8_t value)
{
    return write(address, reg, &value, 1);
}
