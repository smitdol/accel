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
AccelStepper stepper1(FULLSTEP, 8, 10, 9, 11); // was HALFSTEP, which doubles the step count pre rev or halves the RPM?
AccelStepper stepper2(FULLSTEP, 4, 6, 5, 7);

void setup() {
	// set the maximum speed, acceleration factor,
	// initial speed and the target position for motor 1
	stepper1.setMaxSpeed(MAXSPEED);
  // 	_maxSpeed = 1000.0 =>	_cmin = 1000000.0 / speed = SPEED = duration of a step == 1ms
	stepper1.setSpeed(SPEED);

	stepper1.setAcceleration(ACCEL);
  ////// ACCELERATION = Steps /second^2
  //// SPEED = Steps / second  
  // stepInterval = fabs(1000000.0 / speed) = 'duration of a step = 5000us = 5ms => 2038 steps in 10.19 s => ~ 6 rpm
  // speed = MaxSpeed, then duration of a step = 1000us = 1ms => 2038 steps in 2.04 s => ~ 30 rpm ..?
  stepper1.moveTo(2038);

	// set the same for motor 2
	stepper2.setMaxSpeed(MAXSPEED);
	stepper2.setSpeed(SPEED);
	stepper2.setAcceleration(ACCEL);
  stepper2.moveTo(-2038);
}

void loop() {
	// Change direction once the motor reaches target position
	if (stepper1.distanceToGo() == 0) 
		stepper1.moveTo(-stepper1.currentPosition());
	if (stepper2.distanceToGo() == 0) 
		stepper2.moveTo(-stepper2.currentPosition());

	// Move the motor one step
	stepper1.run();
 	stepper2.run();
}
