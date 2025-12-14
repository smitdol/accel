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

//https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037/46
//https://forum.arduino.cc/t/class-and-constructor/653134/11
MyAccelStepper::MyAccelStepper(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable):
AccelStepper(AccelStepper::FULL4WIRE, pin1, pin2, pin3, pin4, enable),
_portMapper1(Mapper(pin1)),
_portMapper2(Mapper(pin2)),
_portMapper3(Mapper(pin3)),
_portMapper4(Mapper(pin4))
{};

void MyAccelStepper::begin(){
  _portMapper1.begin();
  _portMapper2.begin();
  _portMapper3.begin();
  _portMapper4.begin();

}

void MyAccelStepper::disableOutputs() {
  setOutputPins(0);
}

void MyAccelStepper::enableOutputs()
{
 	uint8_t oldSREG = SREG;
	cli();
  _portMapper1.enableOutput();
  _portMapper2.enableOutput();
  _portMapper3.enableOutput();
  _portMapper4.enableOutput();
  SREG = oldSREG;

}


void MyAccelStepper::setOutputPins(uint8_t bits)
{
  uint8_t oldSREG = SREG;
	cli();
	_portMapper1.setOutputPin((bits & (1<<0)));
  _portMapper2.setOutputPin((bits & (1<<1)));
  _portMapper3.setOutputPin((bits & (1<<2)));
  _portMapper4.setOutputPin((bits & (1<<3)));
  SREG = oldSREG;

}
