#pragma once

#include <stdint.h>

enum : uint32_t {
    PA0, PA1, PA2, PA3, PA5, PA8, PA9, PA10,
    PB4, PB5, PB8, PB9, PB12, PB13, PB14, PB15, PC13
};
constexpr uint32_t USER_BTN = PA0;
constexpr uint32_t LED_BUILTIN = PC13;
enum PinMode { OUTPUT_OPEN_DRAIN };
enum PinStatus { LOW, HIGH };

uint32_t millis();
uint32_t micros();
void pinMode(uint32_t pin, PinMode mode);
void digitalWrite(uint32_t pin, PinStatus status);
