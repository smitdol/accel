
#ifndef MyAccelStepper_h
#define MyAccelStepper_h
#include "IMyAccelStepper.h"
#include "AccelStepper.h"
#include "Mapper.h"

class MyAccelStepper : public AccelStepper, public IMyAccelStepper
{
public:
    virtual ~MyAccelStepper() = default;
    MyAccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable = true);
    void begin() override;
    void setOutputPins(uint8_t mask) override;
    void disableOutputs() override;
    void enableOutputs() override;
    void setAcceleration(float accel) override;
    void setMaxSpeed(float speed) override; 
    unsigned long computeNewSpeed() override;

    void setCurrentPosition(long position) override{AccelStepper::setCurrentPosition(position);};
    void move(long relative) override {AccelStepper::move(relative);};
    void setSpeed(float speed) override {AccelStepper::setSpeed(speed);};

    boolean run() override { return AccelStepper::run();};
    long currentPosition() override { return AccelStepper::currentPosition();};
    void moveTo(long absolute) override {AccelStepper::moveTo(absolute);};
    void step4(long step) override {AccelStepper::step4(step);};

private:
  Mapper _portMapper1;
  Mapper _portMapper2;
  Mapper _portMapper3;
  Mapper _portMapper4;

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
