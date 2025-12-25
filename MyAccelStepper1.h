
#ifndef MyAccelStepper1_h
#define MyAccelStepper1_h
#include "IMyAccelStepper.h"
#include "AccelStepper.h"
class MyAccelStepper1 : public AccelStepper, public IMyAccelStepper
{
public:
    virtual ~MyAccelStepper1() = default;
    MyAccelStepper1(const uint8_t interface, const uint8_t digitalPin, bool enable = true);

    void begin() override;
    void setOutputPins(uint8_t mask) override;
    void disableOutputs() override;
    void enableOutputs() override;
    void step4(long step) override;  
    unsigned long computeNewSpeed() override;
    void setAcceleration(float accel) override;
    void setMaxSpeed(float speed) override; 

    void setCurrentPosition(long position) override {AccelStepper::setCurrentPosition(position);};
    void move(long relative) override {AccelStepper::move(relative);};
    void setSpeed(float speed) override {AccelStepper::setSpeed(speed);};
    boolean run() override { return AccelStepper::run();};
    long currentPosition() override { return AccelStepper::currentPosition();};
    void moveTo(long absolute) override  {AccelStepper::moveTo(absolute);};
private:
  uint8_t _port;
  uint8_t _digitalPin;
  uint8_t _mask;
  uint8_t _notMask;
  volatile uint8_t *_out;
  volatile uint8_t *_reg;
  uint8_t _shiftBits;

  /// The step counter for speed calculations
  long _n;

  /// Initial step size in microseconds
  long _c0;

  /// Last step size in microseconds
  long _cn;

  /// Min step size in microseconds based on maxSpeed
  long _cmin; // at max speed

};
#endif
