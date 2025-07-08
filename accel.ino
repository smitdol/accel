// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include <AccelStepper.h>
#define FULLSTEP 4
#include <EEPROM.h>

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

#define totalsteps 14
volatile long pattern[totalsteps][totalsteppers] = {
  //    0,     1,     2,     3,     4,     5,     5,     7,     8,     9,    10,   11,    12,     13,    14,    15 
  {     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
  {  1019,  1019,  1019,  1019,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
  {  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,     0,     0,     0,     0,     0,     0,     0,     0},
  {  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,     0,     0,     0,     0},
  {  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019},
  {  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019},
  {     0,     0,     0,     0,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019},
  {     0,     0,     0,     0,     0,     0,     0,     0,  1019,  1019,  1019,  1019,  1019,  1019,  1019,  1019},
  {     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  1019,  1019,  1019,  1019},
  {     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
  {  -512,  -512,  -512,  -512,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
  {     0,     0,     0,     0,  -512,  -512,  -512,  -512,     0,     0,     0,     0,     0,     0,     0,     0},
  {     0,     0,     0,     0,     0,     0,     0,     0,  -512,  -512,  -512,  -512,     0,     0,     0,     0},
  {     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  -512,  -512,  -512,  -512},
};

void setup() {
  Serial.begin(9600); //define baud rate
  Serial.println("Moin Moin"); //print a message

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
  for(i = 0; i < totalsteppers; i+=4){
    home(i, 4);
  }
  moresteps = false; //restorePositions(); // init next step or continue where left
  step=totalsteps-1; //++step%totalsteps = 0
//  attachInterrupt(digitalPinToInterrupt(PD_PIN), PD_ISR,FALLING); // Set-up Interrupt Service Routine (ISR)
  
}

void nextstep() {
  step=(++step)%totalsteps;
}

void home(unsigned i, unsigned j) {
  //https://curiousscientist.tech/blog/accelstepper-tb6600-homing
  if (i+j > totalsteppers) {
    Serial.print("Cannot home motor "); Serial.println(i+j);
    return;
  }
  unsigned step = 0;
  Serial.print("Start HOMING motors "); Serial.print(i); Serial.print("-");Serial.println(i+j-1);
  for (unsigned k = i; k < i+j; k++){
    steppers[k]->enableOutputs();
    steppers[k]->setCurrentPosition(0);
    steppers[k]->setAcceleration(100); //defining some low acceleration
    steppers[k]->setMaxSpeed(100); //set speed, 100 for test purposes
    steppers[k]->move(-2048); ////set distance - negative value flips the direction, 2048 > 2038 
  }
  Serial.println("Moving to -2048");
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
  Serial.println("Moving to 512");
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
  Serial.print("Completed HOMING motors "); Serial.print(i); Serial.print("-");Serial.println(i+j-1);
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
        Serial.print("received "); Serial.println(receivedCommand);
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
    return;
  }
  nextstep();
  float longestDuration = 0.0;
  for( i = 0; i < totalsteppers; i++) {
    long distanceToGo = pattern[step][i] - steppers[i]->currentPosition();
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
      long distanceToGo = pattern[step][i] - steppers[i]->currentPosition();
      if (distanceToGo != 0) {
        float speed = distanceToGo / longestDuration;
        steppers[i]->setSpeed(speed);
        steppers[i]->moveTo(pattern[step][i]); //reset's speed
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
