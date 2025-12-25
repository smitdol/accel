#include "MyAccelStepper1.h"
#include <Arduino.h>

//wiring_private.h
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

//wiring_digital.c
static void turnOffPWM(uint8_t timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

MyAccelStepper1::MyAccelStepper1(const uint8_t interface, const uint8_t digitalPin, bool enable)
: AccelStepper(interface, digitalPin, digitalPin+1, digitalPin+2, digitalPin+3, enable),
_digitalPin(digitalPin),
_mask(0),
_shiftBits(0),
_n(0),
_c0(0),
_cn(00),
_cmin(1)
{
};

void MyAccelStepper1::begin(){
  _port = digitalPinToPort(_digitalPin);
  _out = portOutputRegister(_port);
  _reg = portModeRegister(_port);
  uint8_t timer;
  for (uint8_t i = 0; i< 4; i++)
  {
    timer = digitalPinToTimer(_digitalPin+i);
    if (timer != NOT_ON_TIMER) turnOffPWM(timer);
    _mask |= digitalPinToBitMask(_digitalPin+i);
  }
  switch (_mask) {
    case 0b11110000: _shiftBits = 4; break;
    case 0b01111000: _shiftBits = 3; break;
    case 0b00111100: _shiftBits = 2; break;
    case 0b00011110: _shiftBits = 1; break;
    case 0b00001111: _shiftBits = 0; break;
    default: _shiftBits = 0; break;
  }
  _notMask = ~_mask;
  enableOutputs();//pinmode output
}

void MyAccelStepper1::disableOutputs() {
	uint8_t oldSREG = SREG;
	cli();
	*_out &= _notMask;
  SREG = oldSREG;
}

void MyAccelStepper1::enableOutputs()
{
  uint8_t oldSREG = SREG;
  cli();
	*_reg |= _mask; //pinmode output
  SREG = oldSREG;
}

void MyAccelStepper1::setOutputPins(uint8_t bits)
{
  uint8_t j = (bits << _shiftBits);
	uint8_t oldSREG = SREG;
	cli();
	*_out |= j;
	*_out &= (_notMask | j);
  SREG = oldSREG;
  /*
                    *out = ???????? ???????? ????????
                mask       11110000 00001111 01111000
                bits       0110         0110  0110
                       j = 01100000 00000110 00110000
   =>          *out |  j = ?11????? ?????11? ??11????

            notMask        00001111 11110000 10000111
   =>                 ~j = 10011111 11111001 11001111
   =>          mask & ~j = 10010000 00001001 01001000
   ->        ~(mask & ~J)= 01101111 11110110 10110111
            _notMask | j = 01101111 11110110 10110111
    *out & (_notMask | j)= 0??0???? ????0??0 ?0??0??? 
   -> *out & ~(mask& ~j) = 0??0???? ????0??0 ?0??0???

	*/
  
}

void MyAccelStepper1::step4(long step)
{
  // bits 1&2 are swapped wrt original
  switch (step & 0x3)
  {
    case 0:    // 1010 => 1100
      setOutputPins(0b1100);
      break;

    case 1:    // 0110 => 0110
      setOutputPins(0b0110);
      break;

    case 2:    //0101 => 0011
      setOutputPins(0b0011);
      break;

    case 3:    //1001 => 1001
      setOutputPins(0b1001);
      break;
  }
}

void MyAccelStepper1::setAcceleration(float acceleration)
{
	// New c0 per Equation 7, with correction per Equation 15
  _c0 = (long)(0.676 * sqrt(2.0 / acceleration) * 1000000.0); // Equation 15
  //_c0 = 39028;// accelleration = 600
	AccelStepper::setAcceleration(acceleration);
}

void MyAccelStepper1::setMaxSpeed(float speed)
{
	_cmin = (long)(1000000.0 / speed);
  AccelStepper::setMaxSpeed( speed);
}

unsigned long MyAccelStepper1::computeNewSpeed()
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

