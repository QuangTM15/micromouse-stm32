#pragma once

#include <stddef.h>
#include <stdint.h>

namespace i2c
{
bool begin();
bool read(uint8_t address, uint8_t reg, uint8_t* data, size_t length);
bool write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length);
bool write(uint8_t address, uint8_t reg, uint8_t value);
}
