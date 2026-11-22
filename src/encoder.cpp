#include "encoder.h"
#include "config.h"

namespace
{
volatile uint32_t leftCount = 0;
volatile uint32_t rightCount = 0;
static_assert(board::leftEncoderSign == 1 || board::leftEncoderSign == -1);
static_assert(board::rightEncoderSign == 1 || board::rightEncoderSign == -1);

void onLeftEdge()
{
    leftCount += encoderEdgeDelta(digitalReadFast(digitalPinToPinName(board::leftEncoderB)) != LOW,
                                  board::leftEncoderSign);
}

void onRightEdge()
{
    rightCount += encoderEdgeDelta(digitalReadFast(digitalPinToPinName(board::rightEncoderB)) != LOW,
                                   board::rightEncoderSign);
}
}

bool encoders::begin()
{
    const auto leftInterrupt = digitalPinToInterrupt(board::leftEncoderA);
    const auto rightInterrupt = digitalPinToInterrupt(board::rightEncoderA);
    if (leftInterrupt == NOT_AN_INTERRUPT || rightInterrupt == NOT_AN_INTERRUPT ||
        STM_PIN(digitalPinToPinName(board::leftEncoderA)) == STM_PIN(digitalPinToPinName(board::rightEncoderA)))
        return false;

    pinMode(board::leftEncoderA, INPUT_PULLUP);
    pinMode(board::leftEncoderB, INPUT_PULLUP);
    pinMode(board::rightEncoderA, INPUT_PULLUP);
    pinMode(board::rightEncoderB, INPUT_PULLUP);
    attachInterrupt(leftInterrupt, onLeftEdge, RISING);
    attachInterrupt(rightInterrupt, onRightEdge, RISING);
    return true;
}

EncoderCounts encoders::read()
{
    const uint32_t interruptMask = __get_PRIMASK();
    __disable_irq();
    const EncoderCounts result{leftCount, rightCount};
    __set_PRIMASK(interruptMask);
    return result;
}
