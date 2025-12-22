
#ifndef MyAccelStepper1_h
#define MyAccelStepper1_h
#include "IMyAccelStepper.h"
#include <AccelStepper.h>
class MyAccelStepper1 : public AccelStepper, public IMyAccelStepper
{
public:
    virtual ~MyAccelStepper1() = default;
    MyAccelStepper1(const uint8_t interface, const uint8_t digitalPin, bool reverse = false, bool enable = true);

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
  uint8_t _port;
  uint8_t _digitalPin;
  uint8_t _mask;
  volatile uint8_t *_out;
  volatile uint8_t *_reg;
  bool _reverse;
  uint8_t _bits[4];

};
#endif
