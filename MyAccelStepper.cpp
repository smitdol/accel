#include "MyAccelStepper.h"
#include "Mapper.h"
#include <Arduino.h>

//https://synthiam.com/Community/Questions/Improving-Digital-Write-Efficiency-Arduino-23032
/*
inline void fastDigitalWrite(uint8_t pin, uint8_t val) {
  uint8_t bit = digitalPinToBitMask(pin);/home/sofie/Downloads/projects/Arduino/gijs/accel/Mapper.h
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out = portOutputRegister(port);
  if (val) {
    *out |= bit;
  } else {
    *out &= ~bit;
  }
}
*/
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


//https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037/46
//https://forum.arduino.cc/t/class-and-constructor/653134/11
MyAccelStepper::MyAccelStepper(const uint8_t pin1, const uint8_t pin2, const uint8_t pin3, const uint8_t pin4, bool enable):
AccelStepper(AccelStepper::FULL4WIRE, pin1, pin2, pin3, pin4, enable),
_portMapper1(Mapper(pin1)),
_portMapper2(Mapper(pin2)),
_portMapper3(Mapper(pin3)),
_portMapper4(Mapper(pin4))
{};

void MyAccelStepper::setOutputPins(uint8_t bits){
  //digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
 	uint8_t oldSREG = SREG;
	cli();
	_portMapper1.setOutputPin(bits & 0b00001);
  _portMapper2.setOutputPin(bits & 0b00010);
  _portMapper3.setOutputPin(bits & 0b00100);
  _portMapper4.setOutputPin(bits & 0b01000);
  SREG = oldSREG;
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