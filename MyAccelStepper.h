
#ifndef MyAccelStepper_h
#define MyAccelStepper_h
#include "IMyAccelStepper.h"
#include <AccelStepper.h>
#include "Mapper.h"

class MyAccelStepper : public AccelStepper, public IMyAccelStepper
{
public:
    virtual ~MyAccelStepper() = default;
    MyAccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool reverse = false, bool enable = true);
    void begin() override;
    void setOutputPins(uint8_t mask) override;
    void disableOutputs() override;
    void enableOutputs() override;

    void setCurrentPosition(long position) override;
    void setAcceleration(float accel) override;
    void setMaxSpeed(float speed) override; 
    void move(long relative) override;
    void setSpeed(float speed) override;

    boolean run() override;
    long currentPosition() override;
    void moveTo(long absolute) override; 
private:

  Mapper _portMapper1;
  Mapper _portMapper2;
  Mapper _portMapper3;
  Mapper _portMapper4;
  int8_t _direction;
};

#endif
