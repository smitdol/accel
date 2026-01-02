// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include "MyAccelStepper1.h"
#include "MyAccelStepper.h"
#define FULLSTEP 4
#include <EEPROM.h>
#define DMX_USE_PORT1
#include <DMXSerial.h>
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

#define totalsteppers 16

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
MyAccelStepper  stepper0(FULLSTEP,  2,  4,  3,  5); //PE4/PE5/PG5/PE3
MyAccelStepper  stepper1(FULLSTEP,  6,  8,  7,  9); //PH3/PH5/PH4/PH6
MyAccelStepper1 stepper2(FULLSTEP, 10); //PB4-7
MyAccelStepper  stepper3(FULLSTEP, 14, 16, 15, 17); //PJ1/PH1/PJ0/PH0
//AccelStepper stepper4(FULLSTEP, 18, 20, 19, 21); //Tx1,SDA,RX1,SCL //PD0-PD3
MyAccelStepper1 stepper4(FULLSTEP, 22); //PA0 22, 24, 23, 25
MyAccelStepper1 stepper5(FULLSTEP, 26); //PA4 26, 28, 27, 29
MyAccelStepper1 stepper6(FULLSTEP, 30); //PC7, PC5, PC6, PC4
MyAccelStepper1 stepper7(FULLSTEP, 34); //PC3, PC1, PC2, PC0
MyAccelStepper  stepper8(FULLSTEP, 38, 40, 39, 41); //PD7, PG1, PG2, PG0
MyAccelStepper1 stepper9(FULLSTEP, 42); //PL7, PL5, PL6, PL4
MyAccelStepper1 stepperA(FULLSTEP, 46); //PL3, PL1 PL2, PL0
MyAccelStepper1 stepperB(FULLSTEP, 50); //PB0-3
MyAccelStepper1 stepperC(FULLSTEP, 54); //PF0-3
MyAccelStepper1 stepperD(FULLSTEP, 58); //PF4-7
MyAccelStepper1 stepperE(FULLSTEP, 62); //PK0-3
MyAccelStepper1 stepperF(FULLSTEP, 66); //PK4-7

IMyAccelStepper* steppers[totalsteppers];

//long positions[totalsteppers+1]; //1 extra is used in EEPROM 
long distanceToGo[totalsteppers];
bool moresteps;
bool hasLCD;
//bool targetset;
//bool written = false;
int step;
unsigned long time;
unsigned long now;
unsigned long delta;
//https://arduino.stackexchange.com/questions/16352/measure-vcc-using-1-1v-bandgap ?
//https://www.codrey.com/arduino-projects/arduino-power-down-auto-save/
//int PD_PIN =2; // D2 used for  power-down detection (INT.0)

// 1019 = 2038/2, where 2038 is not 2048 

const int dmxChannel = 1;
uint8_t module = 0;
#define totalsteps 2
volatile uint8_t pattern[totalsteps][totalsteppers] = {
 // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //0
  { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, //1
};

char row1[17];
char row2[17];
char buffer[17];

void LogLine(const char * s) {
  Serial.println(s);
  strcpy(row1, row2);
  snprintf(row2, 16, s);
  memset(buffer,' ',16);
  if (hasLCD) {
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
}

void setup() {
  Serial.begin(9600); //define baud rate
  Serial.println("Moin Moin"); //print a message
  DMXSerial.init(DMXReceiver, -1); // slave mode
  SetDefaultPattern();
  memset(row1,'\0',17);
  memset(row2,'\0',17);
  memset(buffer,'\0',17);

  int status = lcd.begin(LCD_COLS, LCD_ROWS);
	if(status) // non zero status means it was unsuccesful
	{
  	Serial.print("LCD initalization failed: ");
		Serial.println(status);

		// hd44780 has a fatalError() routine that blinks an led if possible
		// begin() failed so blink error code using the onboard LED if possible
		 hasLCD = false;//hd44780::fatalError(status); // does not return
	} else {
    hasLCD= true;
	  lcd.print("Hello, World!");
  }

  EEPROM.get(0, module);
  snprintf(buffer, 16, "EEPROM0=%i",module);
  if (module > 31) module = 0;
  snprintf(buffer, 16, "Module=%i",module);
  LogLine(buffer);


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
    steppers[k]->setAcceleration(ACCEL); //defining some low acceleration
    steppers[k]->setMaxSpeed(MAXSPEED); //set speed, 100 for test purposes
    steppers[k]->move(-2048); ////set distance - negative value flips the direction, 2048 > 2038 
  }
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
    int parsedInt;
    switch (receivedCommand) {
      case 'h':   
        // homing
        parsedInt = Serial.parseInt();
        home(parsedInt,1);
        break;
      case 'i':
        EEPROM.get(0, module);
        snprintf(buffer, 16, "Module=%i",module);
        LogLine(buffer);
        break;
      case 'm':
        parsedInt = Serial.parseInt();
        if (parsedInt < 0 || parsedInt > 31)
        {
          snprintf(buffer, 16, "invalid %i",parsedInt);
          LogLine(buffer);
        } else {
          module=parsedInt;
          EEPROM.put(0, module);
          snprintf(buffer, 16, "setModule %i",module);
          LogLine(buffer);
        }
        break;
      default:
        snprintf(buffer, 16, "received %c",receivedCommand);
        LogLine(buffer);
        break;
    }
  }

}

void SetDefaultPattern() {
  Serial.print("SetDefaultPattern:");
  int index = dmxChannel + (module * totalsteppers);
  for (uint8_t j = 0; j < totalsteppers; j++){
    uint8_t value = pattern[0][j];
    DMXSerial.write(index+j,value);
    Serial.print(value);Serial.print(",");
  } // 34*16 = 544 > 512; 512/16 = 32; dmxChannel ~ 30*16 = 480
  Serial.println();
}


void UpdatePattern() {
  if (DMXSerial.dataUpdated()) {
    DMXSerial.resetUpdated();
    Serial.print("UpdatePattern:");
    int index = dmxChannel + (module * totalsteppers);
    for (uint8_t j = 0; j < totalsteppers; j++){
      uint8_t value = DMXSerial.read(index+j);
      pattern[0][j]=value;
      Serial.print(value);Serial.print(",");
    }
    Serial.println();
    step = totalsteps-1; // nextstep => step = 0;
  }
}

void loop() {
  CheckSerial();
  if (moresteps) {
    moresteps=false;
    for( uint8_t i = 0; i < totalsteppers; i++) {
      if ( steppers[i]->run())
      {
        moresteps=true;
      }
    }
    return; // loop again
  }
  UpdatePattern();
  now = micros();
  delta = now - time;
  snprintf(buffer,16,"time: %ld  ", delta);
  LogLine(buffer);
  time = now;
  nextstep();

  long longestDistance = 0L;
  for( uint8_t i = 0; i < totalsteppers; i++) {
    long currentpos = steppers[i]->currentPosition();
    if (currentpos <= 0) {
      // home again
      steppers[i]->setCurrentPosition(0);
      steppers[i]->run();
      currentpos = 0;
    }
    distanceToGo[i] = (long)(pattern[step][i]*HOEK) - currentpos; //HOEK
    if (distanceToGo[i] !=  0) {
      longestDistance = max(abs(distanceToGo[i]),longestDistance);
      steppers[i]->enableOutputs();
    } else {
      steppers[i]->disableOutputs();
    }
  }

  if (longestDistance > 0) {
    moresteps = true;
    for( uint8_t i = 0; i < totalsteppers; i++) {
      if (distanceToGo[i] != 0) {
        float speed = ((MAXSPEED*distanceToGo[i]) / (1.0*longestDistance));
        steppers[i]->setSpeed(speed);
        uint8_t value = pattern[step][i];
        if (value == 0) steppers[i]->moveTo(-5); // -5 enforces 0
        else steppers[i]->moveTo(value*HOEK);
      }
    }
  }
}
