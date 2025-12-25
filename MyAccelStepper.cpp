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
