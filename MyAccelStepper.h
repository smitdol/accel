
#ifndef MyAccelStepper_h
#define MyAccelStepper_h

#include <AccelStepper.h>
#include <Arduino.h>

class MyAccelStepper : public AccelStepper
{
public:
    MyAccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, uint8_t pin4 = 5, bool enable = true);
    unsigned long  computeNewSpeed() override;
private:
    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    static long ccache[514];
};

#endif
