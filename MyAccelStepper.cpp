#include "MyAccelStepper.h"
#include "Mapper.h"
#include <Arduino.h>

//https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037/46
//https://forum.arduino.cc/t/class-and-constructor/653134/11
MyAccelStepper::MyAccelStepper(const uint8_t interface, const uint8_t pin1, const uint8_t pin2, const uint8_t pin3, const uint8_t pin4, bool enable)
: AccelStepper(interface, pin1, pin2, pin3, pin4, enable),
_portMapper1(Mapper(pin1)),
_portMapper2(Mapper(pin2)),
_portMapper3(Mapper(pin3)),
_portMapper4(Mapper(pin4))
{
};

void MyAccelStepper::begin(){

  _portMapper1.begin();
  _portMapper2.begin();
  _portMapper3.begin();
  _portMapper4.begin();

}

void MyAccelStepper::disableOutputs() {
  //AccelStepper::disableOutputs();
  setOutputPins(0);
}

void MyAccelStepper::enableOutputs()
{
  //AccelStepper::enableOutputs();
  _portMapper1.enableOutput();
  _portMapper2.enableOutput();
  _portMapper3.enableOutput();
  _portMapper4.enableOutput();

}


void MyAccelStepper::setOutputPins(uint8_t bits)
{
  //AccelStepper::setOutputPins(bits);
	_portMapper1.setOutputPin(bits, 0);
  _portMapper2.setOutputPin(bits, 1);
  _portMapper3.setOutputPin(bits, 2);
  _portMapper4.setOutputPin(bits, 3);
 
}

void MyAccelStepper::setAcceleration(float acceleration)
{
	// New c0 per Equation 7, with correction per Equation 15
  _c0 = (long)(0.676 * sqrt(2.0 / acceleration) * 1000000.0); // Equation 15
  //_c0 = 39028;// accelleration = 600
	AccelStepper::setAcceleration(acceleration);
}

void MyAccelStepper::setMaxSpeed(float speed)
{
	_cmin = (long)(1000000.0 / speed);
  AccelStepper::setMaxSpeed( speed);
}

unsigned long MyAccelStepper::computeNewSpeed()
{
  long distanceTo = AccelStepper::distanceToGo(); // +ve is clockwise from curent location

  long stepsToStop = abs(_n);//(long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16 simpliyfied for this purpose()

  if (distanceTo == 0)// && stepsToStop <= 1)
  {
    // We are at the target and its time to stop
    _stepInterval = 0;
    _speed = 0.0;
    _n = 0;
  } else {

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

    // Need to accelerate or decelerate
    if (_n == 0)
    {
      // First step from stopped
      _cn = _c0;
      _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
      // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
      _cn = _cn - ((2 * _cn) / ((4 * _n) + 1)); // Equation 13 simplified 
      _cn = max(_cn, _cmin); 
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
	    _speed = -_speed;
  }
  return _stepInterval;
}

