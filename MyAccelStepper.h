
#ifndef MyAccelStepper_h
#define MyAccelStepper_h

#include <AccelStepper.h>
#include "Mapper.h"

class MyAccelStepper : public AccelStepper
{
public:
    MyAccelStepper(const uint8_t pin1 , const uint8_t pin2, const uint8_t pin3, const uint8_t pin4, bool enable = false);
    void begin();
    void setOutputPins(uint8_t mask) override;
    void disableOutputs() override;
    void enableOutputs() override;

private:
  Mapper _portMapper1;
  Mapper _portMapper2;
  Mapper _portMapper3;
  Mapper _portMapper4;
};

#endif
