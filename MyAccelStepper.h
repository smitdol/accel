
#ifndef MyAccelStepper_h
#define MyAccelStepper_h

#include <AccelStepper.h>
#include "digitalWriteFast.h"

class MyAccelStepper : public AccelStepper
{
public:
    MyAccelStepper(const uint8_t pin1 , const uint8_t pin2, const uint8_t pin3, const uint8_t pin4, bool enable = true);
    void setOutputPins(uint8_t mask) override;

private:
    uint8_t        _pin1;
    uint8_t        _pin2;
    uint8_t        _pin3;
    uint8_t        _pin4;

    /// Whether the _pins is inverted or not
    uint8_t        _pinInverted[4];
};

#endif
