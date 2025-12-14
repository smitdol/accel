#ifndef Mapper_h
#define Mapper_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

class Mapper
{
public:
  Mapper(const uint8_t pin);
  void begin();
  void setOutputPin(uint8_t val);
  void enableOutput();
private:
  uint8_t _pin;
  uint8_t _bit;
  uint8_t _timer;
  uint8_t _port;
};

#endif

