// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include "MyAccelStepper1.h"
#include "MyAccelStepper.h"
#define FULLSTEP 4
//#include <EEPROM.h>

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Set the maximum steps per second, above 1000 is unreliable according to library
#define SPEED 1024.0
#define MAXSPEED 2048.0
#define ACCEL 600.0
#define HOEK 128 //2048*22.5/360 = 128
#define A -28  // mechanische 0/aanloop/motor op die positie wodt op 0 gezet
#define B HOEK
#define C B+HOEK
#define D C+HOEK
#define E D+HOEK
#ifdef F
#undef F
#endif
#define F E+HOEK
#define G F+HOEK
#define H G+HOEK
#define I H+HOEK //1024 = 180gr
#define J 0 //

#define totalsteppers 16

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
MyAccelStepper stepper0(FULLSTEP,  2,  4,  3,  5); //PE4/PE5/PG5/PE3
MyAccelStepper stepper1(FULLSTEP,  6,  8,  7,  9); //PH3/PH5/PH4/PH6
MyAccelStepper stepper2(FULLSTEP, 10, 12, 11, 13); //PB4/PB6/PB5/PB7
MyAccelStepper stepper3(FULLSTEP, 14, 16, 15, 17); //PJ1/PH1/PJ0/PH0
//AccelStepper stepper4(FULLSTEP, 18, 20, 19, 21); //Tx1,SDA,RX1,SCL //PD0-PD3
MyAccelStepper stepper4(FULLSTEP, 22, 24, 23, 25); //PORTA PA0 PA2 PA1 PA3
//MyAccelStepper stepper5(FULLSTEP, 26, 28, 27, 29); //PORTA PA4 PA6 PA5 PA7
//MyAccelStepper1 stepper4(FULLSTEP, PORTA, 0, 22); //22, 24, 23, 25
MyAccelStepper1 stepper5(FULLSTEP, 26); //26, 28, 27, 29
//MyAccelStepper stepper6(FULLSTEP, 30, 32, 31, 33); //PORTC PC7, PC5, PC6, PC4
//MyAccelStepper stepper7(FULLSTEP, 34, 36, 35, 37); //PORTC PC3, PC1, PC2, PC0
MyAccelStepper1 stepper6(FULLSTEP, 30); //PORTC PC7, PC5, PC6, PC4
MyAccelStepper1 stepper7(FULLSTEP, 34); //PORTC PC3, PC1, PC2, PC0
MyAccelStepper stepper8(FULLSTEP, 38, 40, 39, 41);
//MyAccelStepper stepper9(FULLSTEP, 42, 44, 43, 45); // PORTL, PL7, PL5, PL6, PL4
//MyAccelStepper stepperA(FULLSTEP, 46, 48, 47, 49); // PORTL PL3, PL1 PL2, PL0
MyAccelStepper1 stepper9(FULLSTEP, 42); // PORTL, PL7, PL5, PL6, PL4
MyAccelStepper1 stepperA(FULLSTEP, 46); // PORTL PL3, PL1 PL2, PL0
MyAccelStepper stepperB(FULLSTEP, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_SS);
//MyAccelStepper stepperC(FULLSTEP, A0, A2, A1, A3); //PF0-3
//MyAccelStepper stepperD(FULLSTEP, A4, A6, A5, A7); //PF4-7
MyAccelStepper1 stepperC(FULLSTEP, 54); //PF0-3
MyAccelStepper1 stepperD(FULLSTEP, 58); //PF4-7
//MyAccelStepper stepperE(FULLSTEP, A8, A10, A9, A11); //PK0-3
//MyAccelStepper stepperF(FULLSTEP, A12, A14, A13, A15); //PK4-7
MyAccelStepper1 stepperE(FULLSTEP, 62); //PK0-3
MyAccelStepper1 stepperF(FULLSTEP, 66); //PK4-7

IMyAccelStepper* steppers[totalsteppers];

//long positions[totalsteppers+1]; //1 extra is used in EEPROM 
long distanceToGo[totalsteppers];
bool moresteps;
//bool targetset;
//bool written = false;
int step;
int motornumber = totalsteppers;
unsigned long time;
unsigned long now;
unsigned long delta;
//https://arduino.stackexchange.com/questions/16352/measure-vcc-using-1-1v-bandgap ?
//https://www.codrey.com/arduino-projects/arduino-power-down-auto-save/
//int PD_PIN =2; // D2 used for  power-down detection (INT.0)

// 1019 = 2038/2, where 2038 is not 2048 

#define totalsteps 34
volatile long pattern[totalsteps][totalsteppers] = {
 // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 
  { I, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I}, //0
  { J, I, I, I, I, I, I, I, I, I, I, I, I, I, I, I},
  { I, J, I, I, I, I, I, I, I, I, I, I, I, I, I, I},
  { J, I, J, I, I, I, I, I, I, I, I, I, I, I, I, I},
  { I, J, I, J, I, I, I, I, I, I, I, I, I, I, I, I},
  { J, I, J, I, J, I, I, I, I, I, I, I, I, I, I, I},
  { I, J, I, J, I, J, I, I, I, I, I, I, I, I, I, I},
  { J, I, J, I, J, I, J, I, I, I, I, I, I, I, I, I},
  { I, J, I, J, I, J, I, J, I, I, I, I, I, I, I, I},
  { J, I, J, I, J, I, J, I, J, I, I, I, I, I, I, I},
  { I, J, I, J, I, J, I, J, I, J, I, I, I, I, I, I},
  { J, I, J, I, J, I, J, I, J, I, J, I, I, I, I, I},
  { I, J, I, J, I, J, I, J, I, J, I, J, I, I, I, I},
  { J, I, J, I, J, I, J, I, J, I, J, I, J, I, I, I},
  { I, J, I, J, I, J, I, J, I, J, I, J, I, J, I, I},
  { J, I, J, I, J, I, J, I, J, I, J, I, J, I, J, I},
  { I, J, I, J, I, J, I, J, I, J, I, J, I, J, I, J},
  { J, I, J, I, J, I, J, I, J, I, J, I, J, I, J, I}, //17

  { I, H, A, A, A, A, A, A, A, A, A, A, A, A, A, A}, //18
  { J, A, F, C, B, A, A, A, A, A, A, A, A, A, A, A},
  { H, H, F, C, B, B, B, B, B, B, B, B, A, A, A, A},
  { J, B, B, F, B, B, B, B, B, B, B, B, B, B, B, B},
  { G, H, B, F, B, D, B, D, B, D, B, D, B, D, B, D},
  { J, C, C, D, C, D, C, D, C, D, C, D, C, D, C, D},
  { F, H, C, D, E, E, C, D, E, E, C, D, E, E, C, D},
  { J, D, E, E, E, E, E, E, E, E, E, E, E, E, E, E},
  { E, H, E, E, A, A, A, A, A, A, A, A, E, E, E, E},
  { J, E, A, A, A, A, A, A, A, H, H, H, A, A, A, A},
  { D, H, G, H, F, A, H, H, H, H, H, H, A, A, A, A},
  { J, F, G, A, F, A, A, A, A, A, A, H, A, A, A, A},
  { C, H, G, A, F, A, A, A, A, A, B, A, E, E, E, E},
  { J, G, B, C, D, E, E, D, A, A, B, A, E, E, E, E},
  { B, H, B, B, C, D, D, C, B, A, A, A, E, E, E, E},
  { J, I, B, B, C, D, D, C, B, A, A, A, E, C, C, F},
//  { I, H, B, B, A, A, A, A, A, A, A, A, A, C, C, F}, //34
  };

char row1[17];
char row2[17];
char buffer[17];

void LogLine(const char * s) {
  Serial.println(s);
  strcpy(row1, row2);
  snprintf(row2, 16, s);
  memset(buffer,' ',16);
  Wire.begin();
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print(buffer);  
  lcd.setCursor(0,1);  
  lcd.print(buffer);
  lcd.setCursor(0,0);  
  lcd.print(row1);  
  lcd.setCursor(0,1);  
  lcd.print(row2);
  Wire.end();  
}

void setup() {
  Serial.begin(9600); //define baud rate
  Serial.println("Moin Moin"); //print a message
  memset(row1,'\0',17);
  memset(row2,'\0',17);
  memset(buffer,'\0',17);

  int status;
	status = lcd.begin(LCD_COLS, LCD_ROWS);
	if(status) // non zero status means it was unsuccesful
	{
  	Serial.print("LCD initalization failed: ");
		Serial.println(status);

		// hd44780 has a fatalError() routine that blinks an led if possible
		// begin() failed so blink error code using the onboard LED if possible
		 hd44780::fatalError(status); // does not return
	}
	lcd.print("Hello, World!");

  steppers[0] = &stepper0;
  steppers[1] = &stepper1;
  steppers[2] = &stepper2;
  steppers[3] = &stepper3;
  steppers[4] = &stepper4;
  steppers[5] = &stepper5;
  steppers[6] = &stepper6;
  steppers[7] = &stepper7;
  steppers[8] = &stepper8;
  steppers[9] = &stepper9;
  steppers[10] = &stepperA;
  steppers[11] = &stepperB;
  steppers[12] = &stepperC;
  steppers[13] = &stepperD;
  steppers[14] = &stepperE;
  steppers[15] = &stepperF;

  for(uint8_t i = 0; i < totalsteppers; i++){
    steppers[i]->begin();
  }

  for(uint8_t i = 0; i < totalsteppers; i++){
    steppers[i]->disableOutputs();
  }
  home(0, 8);
  home(8, 8);
  moresteps = false; //restorePositions(); // init next step or continue where left
  step=totalsteps-1; //++step%totalsteps = 0
//  attachInterrupt(digitalPinToInterrupt(PD_PIN), PD_ISR,FALLING); // Set-up Interrupt Service Routine (ISR)
  time = micros();  
}

void nextstep() {
  step=(++step)%totalsteps;
}

void home(unsigned i, unsigned j) {
  //https://curiousscientist.tech/blog/accelstepper-tb6600-homing
  LogLine("Homing motors");
  if (i+j > totalsteppers) {
    LogLine("Cannot home");
    snprintf(buffer,16, "motor %i", i+j);
    LogLine(buffer);
    return;
  }
  snprintf(buffer, 16, "%i - %i", i+1, i+j);
  LogLine(buffer);
  for (unsigned k = i; k < i+j; k++){
    steppers[k]->enableOutputs();
    steppers[k]->setCurrentPosition(0);
    steppers[k]->setAcceleration(100); //defining some low acceleration
    steppers[k]->setMaxSpeed(100); //set speed, 100 for test purposes
    steppers[k]->move(-2048); ////set distance - negative value flips the direction, 2048 > 2038 
  }
  //snprintf(buffer, 16, "Moving to -2048");LogLine(buffer);
  do {
    moresteps = false;
    for (unsigned k = i; k < i+j; k++){
      if (steppers[k]->run()) {
        moresteps = true;
      }
    }
  } while (moresteps);
  for (unsigned k = i; k < i+j; k++){
    steppers[k]->setSpeed(SPEED);
    steppers[k]->setAcceleration(ACCEL);
    steppers[k]->setCurrentPosition(0);
    steppers[k]->setMaxSpeed(MAXSPEED);
    //steppers[k]->run();
    steppers[k]->disableOutputs();
  }
  LogLine("Completed HOMING");
  snprintf(buffer, 16, "motors %i - %i", i+1, i+j);
  LogLine(buffer);
}

void CheckSerial() {
  if (Serial.available() > 0) //if something comes
  {
    char receivedCommand = Serial.read(); // this will read the command character
    switch (receivedCommand) {
      case 'h':   
        // homing
        motornumber = Serial.parseInt();
        home(motornumber,1);
        break;
      default:
        snprintf(buffer, 16, "received %c",receivedCommand);
        LogLine(buffer);
        break;
    }
  }

}

void loop() {
  CheckSerial();
  if (moresteps) {
    moresteps=false;

    for( uint8_t i = 0; i < totalsteppers; i++) {
//time = micros();
      if ( steppers[i]->run())
      {
        moresteps=true;
      }
  //now = micros();
  //delta = now - time;
  //snprintf(buffer,16,"time: %ld  ", delta);
  //LogLine(buffer);
    }

    return; // loop again
  }
  now = micros();
  delta = now - time;
  snprintf(buffer,16,"time: %ld  ", delta);
  LogLine(buffer);
  time = now;
  nextstep();

  long longestDistance = 0.0;
  for( uint8_t i = 0; i < totalsteppers; i++) {
    long currentpos = steppers[i]->currentPosition();
    if (A == currentpos) {
      // home again
      steppers[i]->setCurrentPosition(0);
      steppers[i]->run();
      currentpos = 0;
    }
    distanceToGo[i] = pattern[step][i] - currentpos;
    if (distanceToGo[i] !=  0) {
      longestDistance = max(abs(distanceToGo[i]),longestDistance);
      steppers[i]->enableOutputs();
    } else {
      steppers[i]->disableOutputs();
    }
  }
//  snprintf(buffer,16,"step: %d  ", step);
//  LogLine(buffer);

  if (longestDistance > 0.0) {
    moresteps = true;
    for( uint8_t i = 0; i < totalsteppers; i++) {
      if (distanceToGo[i] != 0) {
        float speed = MAXSPEED*(distanceToGo[i] / longestDistance);
        steppers[i]->setSpeed(speed);
        steppers[i]->moveTo(pattern[step][i]); //reset's speed
//    } else {
//      steppers[i]->disableOutputs();
      }
    }
  }
}
/*
void PD_ISR () { // ISR to be get called on power-down state
//    powerDown();
}

bool powerDown() {
  for(i = 0; i < totalsteppers; i++) {
    steppers[i]->setSpeed(0);
    steppers[i]->disableOutputs();
  }
  savePositions();
}
void savePositions() {
  position=0;
  for(i = 0; i < totalsteppers; i++) {
    positions[i]= steppers[i]->currentPosition();
    position^=positions[i];
  }
  positions[totalsteppers]=position;
  EEPROM.put(0, positions); // takes 3.3 ms per write
  EEPROM.put(sizeof(positions),step);
}

bool restorePositions() {
  EEPROM.get(0, positions); // any position saved?
  position=0;
  for(i = 0; i < totalsteppers; i++) {
    position^=positions[i];
  }

  if (positions[totalsteppers] == position) {
    for(i = 0; i < totalsteppers; i++) {
      steppers[i]->setCurrentPosition(positions[i]);
    }
    EEPROM.get(sizeof(positions),step);
    return true;
  } else {
    for(i = 0; i < totalsteppers; i++){
      steppers[i]->setCurrentPosition(0);
    }
    return false;
  }
}

//https://forum.arduino.cc/t/detecting-battery-voltage-using-bandgap-internal-1-1v-reference/225628/2
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
*/