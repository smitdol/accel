#include "MyAccelStepper.h"

MyAccelStepper::MyAccelStepper(const uint8_t pin1, const uint8_t pin2, const uint8_t pin3, const uint8_t pin4, bool enable):
AccelStepper(AccelStepper::FULL4WIRE, pin1, pin2, pin3, pin4, enable)
{
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
    int i;
    for (i = 0; i < 4; i++)
	    _pinInverted[i] = 0;
    pinModeFast(pin1, OUTPUT);
    pinModeFast(pin2, OUTPUT);
    pinModeFast(pin3, OUTPUT);
    pinModeFast(pin4, OUTPUT);
};
void MyAccelStepper::setOutputPins(uint8_t mask){
	digitalWriteFast(_pin1, (mask & (1 << 0)) ? (HIGH ^ _pinInverted[0]) : (LOW ^ _pinInverted[0]));
	digitalWriteFast(_pin2, (mask & (1 << 1)) ? (HIGH ^ _pinInverted[1]) : (LOW ^ _pinInverted[1]));
	digitalWriteFast(_pin3, (mask & (1 << 2)) ? (HIGH ^ _pinInverted[2]) : (LOW ^ _pinInverted[2]));
	digitalWriteFast(_pin4, (mask & (1 << 3)) ? (HIGH ^ _pinInverted[4]) : (LOW ^ _pinInverted[3]));
}

/*
unsigned long  MyAccelStepper::computeNewSpeed() 
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((speed() * speed()) / (2.0 * acceleration())); // Equation 16

    if (distanceTo == 0)// && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
      setSpeed(0);
      return _stepInterval;
    }

    if (distanceTo > 0)
    {
      // We are anticlockwise from the target
      // Need to go clockwise from here, maybe decelerate now
      if (_n > 0)
      {
          // Currently accelerating, need to decel now? Or maybe going the wrong way?
          if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
            _n = -stepsToStop; // Start deceleration
      }
      else if (_n < 0)
      {
          // Currently decelerating, need to accel again?
          if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
            _n = -_n; // Start accceleration
      }
    }
    else if (distanceTo < 0)
    {
      // We are clockwise from the target
      // Need to go anticlockwise from here, maybe decelerate
      if (_n > 0)
      {
          // Currently accelerating, need to decel now? Or maybe going the wrong way?
          if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
            _n = -stepsToStop; // Start deceleration
      }
      else if (_n < 0)
      {
          // Currently decelerating, need to accel again?
          if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
            _n = -_n; // Start accceleration
      }
    }
    long idx = min(abs(_n), 513);
    _cn = ccache[idx];
    _n++;
    if (_direction == DIRECTION_CCW) {
      _cn = -_cn;
    }
    _stepInterval = _cn;
    setSpeed(1000000.0 / _cn);
    return _stepInterval;
}
*/