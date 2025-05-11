// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>

// Set the maximum steps per second, above 1000 is unreliable according to library
#define SPEED 1000.0
#define MAXSPEED 1000.0
#define ACCEL 200.0

#define totalsteppers 16

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper0( 2,  4,  3,  5, false); // was HALFSTEP, which doubles the step count pre rev or halves the RPM?
AccelStepper stepper1( 6,  8,  7,  9, false);
AccelStepper stepper2(10, 12, 11, 13, false);
AccelStepper stepper3(14, 16, 15, 17, false);
AccelStepper stepper4(18, 20, 19, 21, false);
AccelStepper stepper5(22, 24, 23, 25, false);
AccelStepper stepper6(26, 28, 27, 29, false);
AccelStepper stepper7(30, 32, 31, 33, false);
AccelStepper stepper8(34, 36, 35, 37, false);
AccelStepper stepper9(38, 40, 39, 41, false);
AccelStepper stepperA(42, 44, 43, 45, false);
AccelStepper stepperB(46, 48, 47, 49, false);
AccelStepper stepperC(50, 52, 51, 53, false);
AccelStepper stepperD(54, 56, 55, 57, false);
AccelStepper stepperE(58, 60, 59, 61, false);
AccelStepper stepperF(62, 64, 63, 65, false);


MultiStepper steppers;

volatile long positions[totalsteppers+1]; //1 extra is used in EEPROM 
volatile bool moresteps;
volatile unsigned i;
volatile int eeAdress;
int potpin = 0;  // [0=A0] analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
volatile long position;
volatile int step;

volatile unsigned long previousMillis = 0;  
const long interval = 5000; 


#define totalsteps 9
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
};

void setup() {
  Serial.begin(9600);
  if (steppers.addStepper(stepper0)) Serial.write("0");
  if (steppers.addStepper(stepper1)) Serial.write("1");
  if (steppers.addStepper(stepper2)) Serial.write("2");
  if (steppers.addStepper(stepper3)) Serial.write("3");
  if (steppers.addStepper(stepper4)) Serial.write("4");
  if (steppers.addStepper(stepper5)) Serial.write("5");
  if (steppers.addStepper(stepper6)) Serial.write("6");
  if (steppers.addStepper(stepper7)) Serial.write("7");
  if (steppers.addStepper(stepper8)) Serial.write("8");
  if (steppers.addStepper(stepper9)) Serial.write("9");
  if (steppers.addStepper(stepperA)) Serial.write("A");
  if (steppers.addStepper(stepperB)) Serial.write("B");
  if (steppers.addStepper(stepperC)) Serial.write("C");
  if (steppers.addStepper(stepperD)) Serial.write("D");
  if (steppers.addStepper(stepperE)) Serial.write("E");
  if (steppers.addStepper(stepperF)) Serial.write("F");
  
  for(i = 0; i < steppers._num_steppers; i++) {
    steppers._steppers[i]->setMaxSpeed(MAXSPEED);
    steppers._steppers[i]->setSpeed(SPEED);
    steppers._steppers[i]->setAcceleration(ACCEL);
  }
  restorePositions();
  moresteps = true;
  step=0;
}

void loop() {
  // proper shutdown via serial comm? clear EEPROM in that case
  // serial.read 
  // if x => exit
  // if s => start
  // if c => calibration submenu:
  // if # => select motor # 
  // if + => move slow clockwise
  // if - => move slow ccw
  // x during calibration stops motor and set position to 0
  // if e => erase EEPROM
  // if p => erase positions from EEPROM
  // if u => update programtable (positions, delay) (in EEPROM?)

  moresteps = powerOk();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (powerOk()) {
      steppers.moveTo(pattern[step]);
      while(powerOk() & steppers.run()){
        YIELD; // do housekeeping
      }
    }

    step=(++step)%totalsteps;
  }
}

bool powerOk() {
//  val = analogRead(potpin);
//  if (false & (val < 972)) { // read analog pin that watches over the power, 1023 = 5V, which is 770k/550k (1%) of 12V, 972/1023 ~ 5% drop
//    powerDown();
//    return false;
//  }
  return true;
}

bool powerDown() {
  for(i = 0; i < steppers._num_steppers; i++) {
    steppers._steppers[i]->setSpeed(0);
    steppers._steppers[i]->disableOutputs();
  }
  savePositions();
}
void savePositions() {
  position=0;
  for(i = 0; i < steppers._num_steppers; i++) {
    positions[i]= steppers._steppers[i]->currentPosition();
    position^=positions[i];
  }
  positions[totalsteppers]=position;
  EEPROM.put(0, positions); // takes 3.3 ms per write
}

void restorePositions() {
  EEPROM.get(0, positions); // any position saved?
  position=0;
  for(i = 0; i < steppers._num_steppers; i++) {
    position^=positions[i];
  }

  if (positions[totalsteppers] == position) {
    for(i = 0; i < steppers._num_steppers; i++) {
      steppers._steppers[i]->setCurrentPosition(positions[i]);
    }
  } else {
    for(i = 0; i < steppers._num_steppers; i++){
      steppers._steppers[i]->setCurrentPosition(0);
    }
  }
  for(i = 0; i < steppers._num_steppers; i++){
    positions[i] = 0;
  }
}