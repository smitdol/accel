#ifndef IMyAccelStepper1_h
#define IMyAccelStepper1_h

#include <Arduino.h>

class IMyAccelStepper 
{
public:
    virtual ~IMyAccelStepper() {};
    virtual void begin() = 0;
    virtual void setOutputPins(uint8_t mask) = 0;
    virtual void disableOutputs() = 0;
    virtual void enableOutputs() = 0;
    virtual void setCurrentPosition(long position) = 0;
    virtual void setAcceleration(float accel) = 0;
    virtual void setMaxSpeed(float speed) = 0; 
    virtual void move(long relative) = 0;
    virtual void setSpeed(float speed) = 0;
    virtual boolean run() = 0;
    virtual long currentPosition() = 0;
    virtual void moveTo(long absolute) = 0;   
};
#endif