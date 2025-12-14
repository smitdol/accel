
#ifndef MyAccelStepper_h
#define MyAccelStepper_h

#include <AccelStepper.h>
#include "Mapper.h"

class MyAccelStepper : public AccelStepper
{
public:
    MyAccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, uint8_t pin4 = 5, bool enable = true);
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
