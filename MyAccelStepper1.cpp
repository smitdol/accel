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

MyAccelStepper1::MyAccelStepper1(const uint8_t interface, const uint8_t digitalPin, bool reverse, bool enable)
: AccelStepper(interface, digitalPin, digitalPin+1, digitalPin+2, digitalPin+3, enable),
_digitalPin(digitalPin),
_mask(0),
_shiftBits(0)
{
  if (reverse) {
    _direction = -1;
  } else {
    _direction = +1;
  }
};

void MyAccelStepper1::begin(){

  _port = digitalPinToPort(_digitalPin);
  _out = portOutputRegister(_port);
  _reg = portModeRegister(_port);
  uint8_t timer;
  for (uint8_t i = 0; i< 4; i++)
  {
    timer = digitalPinToTimer(_digitalPin+i);
    _bits[i] = digitalPinToBitMask(_digitalPin+i);

    if (timer != NOT_ON_TIMER) turnOffPWM(timer);
    _mask |= _bits[i];
  }
  switch (_mask) {
    case 0b11110000: _shiftBits = 4; break;
    case 0b01111000: _shiftBits = 3; break;
    case 0b00111100: _shiftBits = 2; break;
    case 0b00011110: _shiftBits = 1; break;
    case 0b00001111: _shiftBits = 0; break;
  }
  enableOutputs();//pinmode output

}

void MyAccelStepper1::disableOutputs() {
  setOutputPins(0);
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
	*_out &= ~( _mask & ~j);

  /*
                    *out = ???????? ????????
                mask       11110000 00001111
                bits       0110         0110
                       j = 01100000 00000110
   =>          *out |  j = ?11????? ?????11?

   =>                 ~j = 10011111 11111001
   =>          mask & ~j = 10010000 00001001
   ->        ~(mask & ~) = 01101111 11110110
   -> *out & ~(mask& ~j) = 0??0???? ????0??0

	*/
  SREG = oldSREG;
  
}
void MyAccelStepper1::setCurrentPosition(long position) {AccelStepper::setCurrentPosition(_direction*position);}
void MyAccelStepper1::setAcceleration(float accel) {AccelStepper::setAcceleration(accel);}
void MyAccelStepper1::setMaxSpeed(float speed) {AccelStepper::setMaxSpeed(speed);}
void MyAccelStepper1::move(long relative) {AccelStepper::move(_direction*relative);}
void MyAccelStepper1::setSpeed(float speed) {AccelStepper::setSpeed(speed);}
boolean MyAccelStepper1::run() { return AccelStepper::run();}
long MyAccelStepper1::currentPosition() { return _direction* AccelStepper::currentPosition();}
void MyAccelStepper1::moveTo(long absolute) {AccelStepper::moveTo(_direction*absolute);}