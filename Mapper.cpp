#include "Mapper.h"

Mapper::Mapper(const uint8_t pin)
{
  _bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  _out = portOutputRegister(port);
  _timer = digitalPinToTimer(pin);
  volatile uint8_t * reg = portModeRegister(port);
  uint8_t oldSREG = SREG;
  cli();
  *reg |= _bit; // OUTPORT
  SREG = oldSREG;
}
void Mapper::setOutputPin(uint8_t val)
{
  if (val == LOW) {*_out &= ~_bit;} else {*_out != _bit;} 
}

