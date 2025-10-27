// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include <AccelStepper.h>
#define FULLSTEP 4
#include <EEPROM.h>

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Set the maximum steps per second, above 1000 is unreliable according to library
#define SPEED 1000.0
#define MAXSPEED 1000.0
#define ACCEL 200.0

#define totalsteppers 16

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper0(FULLSTEP,  2,  4,  3,  5); // was HALFSTEP, which doubles the step count pre rev or halves the RPM?
AccelStepper stepper1(FULLSTEP,  6,  8,  7,  9);
AccelStepper stepper2(FULLSTEP, 10, 12, 11, 13);
AccelStepper stepper3(FULLSTEP, 14, 16, 15, 17);
AccelStepper stepper4(FULLSTEP, 18, 20, 19, 21);
AccelStepper stepper5(FULLSTEP, 22, 24, 23, 25);
AccelStepper stepper6(FULLSTEP, 26, 28, 27, 29);
AccelStepper stepper7(FULLSTEP, 30, 32, 31, 33);
AccelStepper stepper8(FULLSTEP, 34, 36, 35, 37);
AccelStepper stepper9(FULLSTEP, 38, 40, 39, 41);
AccelStepper stepperA(FULLSTEP, 42, 44, 43, 45);
AccelStepper stepperB(FULLSTEP, 46, 48, 47, 49);
AccelStepper stepperC(FULLSTEP, 50, 52, 51, 53);
AccelStepper stepperD(FULLSTEP, 54, 56, 55, 57);
AccelStepper stepperE(FULLSTEP, 58, 60, 59, 61);
AccelStepper stepperF(FULLSTEP, 62, 64, 63, 65);

AccelStepper* steppers[totalsteppers];

long positions[totalsteppers+1]; //1 extra is used in EEPROM 
bool moresteps;
bool targetset;
bool written = false;
unsigned i;
int eeAdress;
long position;
int step;
int motornumber = totalsteppers;
//https://arduino.stackexchange.com/questions/16352/measure-vcc-using-1-1v-bandgap ?
//https://www.codrey.com/arduino-projects/arduino-power-down-auto-save/
//int PD_PIN =2; // D2 used for  power-down detection (INT.0)

// 1019 = 2038/2, where 2038 is not 2048 

#define totalsteps 17
volatile long pattern[totalsteps][totalsteppers] = {
 // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 
  { 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3},
  { 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3},
  { 4, 4, 2, 3, 4, 4, 2, 3, 4, 4, 2, 3, 4, 4, 2, 3},
  { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},
  { 0, 1, 2, 3, 4, 4, 4, 4, 3, 2, 1, 0, 4, 4, 4, 4},
  { 0, 0, 1, 2, 3, 4, 4, 3, 2, 1, 0, 0, 4, 4, 4, 4},
  { 0, 0, 0, 1, 2, 3, 3, 2, 1, 0, 0, 0, 4, 4, 4, 4},
  { 0, 0, 0, 0, 1, 2, 2, 1, 0, 0, 0, 0, 4, 2, 2, 4},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0},
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
  { 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

char row1[17];
char row2[17];
char buffer[17];

void LogLine(const char * s) {
  Serial.println(s);
  strcpy(row1, row2);
  snprintf(row2, 16, s);
  Wire.begin();
  lcd.clear();
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

  for(i = 0; i < totalsteppers; i++){
    steppers[i]->disableOutputs();
  }
  for(i = 0; i < totalsteppers; i+=8){
    home(i, 8);
  }
  moresteps = false; //restorePositions(); // init next step or continue where left
  step=totalsteps-1; //++step%totalsteps = 0
//  attachInterrupt(digitalPinToInterrupt(PD_PIN), PD_ISR,FALLING); // Set-up Interrupt Service Routine (ISR)
  
}

void nextstep() {
  step=(++step)%totalsteps;
  snprintf(buffer,16,"Next step: %i  ", step);
  LogLine(buffer);
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
  unsigned step = 0;
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
    steppers[k]->setMaxSpeed(MAXSPEED);
    steppers[k]->setSpeed(SPEED);
    steppers[k]->setAcceleration(ACCEL);
    steppers[k]->move(512); // off-set = 2048/4
  }
  snprintf(buffer, 16, "Moving to 512");LogLine(buffer);
  do {
    moresteps = false;
    for (unsigned k = i; k < i+j; k++){
      if (steppers[k]->run()) {
        moresteps = true;
      }
    }
  } while (moresteps);
  for (unsigned k = i; k < i+j; k++){
    steppers[k]->setCurrentPosition(0);
    steppers[k]->run();
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
    moresteps = false;
    for( i = 0; i < totalsteppers; i++) {
      if (steppers[i]->run()) {
        moresteps = true; // loop again
      }
    }
    return; // loop again
  }
  nextstep();
  float longestDuration = 0.0;
  for( i = 0; i < totalsteppers; i++) {
    long distanceToGo = pattern[step][i]*512 - steppers[i]->currentPosition();
    float duration = abs(distanceToGo)/MAXSPEED;
    if (duration > 0.0) {
      steppers[i]->enableOutputs();
    } else {
      steppers[i]->disableOutputs();
    }
    longestDuration = max(duration,longestDuration);
  }
  if (longestDuration > 0.0) {
    moresteps = true;
    for( i = 0; i < totalsteppers; i++) {
      long distanceToGo = pattern[step][i]*512 - steppers[i]->currentPosition();
      if (distanceToGo != 0) {
        float speed = distanceToGo / longestDuration;
        steppers[i]->setSpeed(speed);
        steppers[i]->moveTo(pattern[step][i]*512); //reset's speed
      } else {
 //       steppers[i]->disableOutputs();
      }
    }
  }
}

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
