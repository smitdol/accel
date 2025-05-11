// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// Include the AccelStepper Library (Mike McCauley)
#include <AccelStepper.h>

// Define step constants
#define FULLSTEP 4
#define HALFSTEP 8
#define RSTEPS 2038

// Set the maximum steps per second, above 1000 is unreliable according to library
#define SPEED 1000.0
#define MAXSPEED 1000.0
#define ACCEL 200.0

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

AccelStepper* steppers[16];



void setup() {
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
	volatile unsigned int i;
  for(i = 0; i <= 15; i++){
    steppers[i]->setMaxSpeed(MAXSPEED);
    steppers[i]->setSpeed(SPEED);
    steppers[i]->setAcceleration(ACCEL);
  }
  for(i = 0; i <= 15; i++)
  {
    steppers[i]->enableOutputs();
    steppers[i]->moveTo(RSTEPS);
  }
  for ( i = 4; i < 15; i++)
  {
    //steppers[i]->disableOutputs();
    //steppers[i]->moveTo(0);
  }
}

void loop() {
	// Change direction once the motor reaches target position
  volatile unsigned int i;
  volatile unsigned int j;
  for( i = 0; i <= 15; i++) {
    j = i;//(i+4) & 15;
    if (steppers[i]->distanceToGo() == 0) 
    {
      steppers[j]->enableOutputs();
      steppers[j]->moveTo(-steppers[i]->currentPosition());
      //steppers[i]->disableOutputs();
    } 
  } 
    // Move the motor one step
  for(i = 0; i <= 15; i++) {
      steppers[i]->run();
  }
}
