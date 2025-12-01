import math
#import time
from typing import Optional, Tuple, List

class AccelStepper:
    DIRECTION_CCW = 0
    DIRECTION_CW = 1

    def __init__(self, enable: bool = False):
        self._current_pos = 0
        self._target_pos = 0
        self._speed = 0.0
        self._max_speed = 0.0
        self._acceleration = 0.0
        self._sqrt_twoa = 1.0
        self._step_interval = 0
        self._min_pulse_width = 1
        self._enable_pin = 0xff
        self._last_step_time = 0
        self._enable_inverted = False

        # NEW variables
        self._n = 0
        self._c0 = 0.0
        self._cn = 0.0
        self._cmin = 1.0
        self._direction = self.DIRECTION_CCW
        self._printHeader = True

    def move_to(self, absolute: int) -> None:
        """Set the target position in absolute steps."""
        if self._target_pos != absolute:
            self._target_pos = absolute
            self._compute_new_speed()

    def move(self, relative: int) -> None:
        """Set the target position in relative steps from current position."""
        self.move_to(self._current_pos + relative)

    def run_speed(self) -> bool:
        """Implements steps according to the current step interval.
        Returns True if a step occurred, False otherwise."""
        if not self._step_interval:
            return False

#        current_time = time.time_ns() // 1000  # Convert to microseconds
#        if current_time - self._last_step_time >= self._step_interval:
        if self._direction == self.DIRECTION_CW:
            self._current_pos += 1
        else:
            self._current_pos -= 1

 #       self._step(self._current_pos)
 #       self._last_step_time = current_time
        return True
 #       return False

    def distance_to_go(self) -> int:
        """Returns the distance from current position to target position."""
        return self._target_pos - self._current_pos

    @property
    def target_position(self) -> int:
        """Returns the target position."""
        return self._target_pos

    @property
    def current_position(self) -> int:
        """Returns the current position."""
        return self._current_pos

    def set_current_position(self, position: int) -> None:
        """Sets the current position and resets speed-related variables."""
        self._target_pos = position
        self._current_pos = position
        self._n = 0
        self._step_interval = 0
        self._speed = 0.0

    def _compute_new_speed(self) -> int:
        """Computes new speed based on acceleration and distance to target.
        Returns the new step interval."""
        distance_to = self.distance_to_go()
        steps_to_stop = int((self._speed * self._speed) / (2.0 * self._acceleration))  # Equation 16

        if distance_to == 0 and steps_to_stop <= 1:
            # We are at the target and it's time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            return self._step_interval

        if distance_to > 0:
            # Need to go clockwise from here, maybe decelerate now
            if self._n > 0:
                if (steps_to_stop >= distance_to) or (self._direction == self.DIRECTION_CCW):
                    self._n = -steps_to_stop  # Start deceleration
            elif self._n < 0:
                if (steps_to_stop < distance_to) and (self._direction == self.DIRECTION_CW):
                    self._n = -self._n  # Start acceleration
        elif distance_to < 0:
            # Need to go anticlockwise from here, maybe decelerate
            if self._n > 0:
                if (steps_to_stop >= -distance_to) or (self._direction == self.DIRECTION_CW):
                    self._n = -steps_to_stop  # Start deceleration
            elif self._n < 0:
                if (steps_to_stop < -distance_to) and (self._direction == self.DIRECTION_CCW):
                    self._n = -self._n  # Start acceleration

        # Need to accelerate or decelerate
        if self._n == 0:
            # First step from stopped
            self._cn = self._c0
            self._direction = self.DIRECTION_CW if distance_to > 0 else self.DIRECTION_CCW
        else:
            # Subsequent step. Works for accel (n is +ve) and decel (n is -ve)
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1))  # Equation 13
            self._cn = max(self._cn, self._cmin)

        self._n += 1
        self._step_interval = int(self._cn)
        self._speed = 1000000.0 / self._cn
        if self._direction == self.DIRECTION_CCW:
            self._speed = -self._speed

        return self._step_interval

    def run(self) -> bool:
        """Run the motor to implement speed and acceleration to reach target position.
        Returns True if the motor is still running to the target position."""
        if self.run_speed():
            self._compute_new_speed()
        return self._speed != 0.0 or self.distance_to_go() != 0

    def setCurrentPosition(self, position: int) -> None:
        self._target_pos = position
        self._current_pos = position
        self._n = 0
        self._stepInterval = 0
        self_speed = 0.0

    def setAcceleration(self, acceleration: float) -> None:
        if acceleration == 0.0:
            return;
        if acceleration < 0.0:
            acceleration = -acceleration
        if self._acceleration != acceleration:
            #Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
        # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0; # Equation 15
            self._acceleration = acceleration
            self._compute_new_speed()

    def setMaxSpeed(self, speed: float) -> None:
        if speed < 0.0:
            speed = -speed;
        if self._max_speed != speed:
            self._max_speed = speed
            self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if self._n > 0:
                self._n = (long)((self._speed * self._speed) / (2.0 * self._acceleration)) # Equation 16
                self._compute_new_speed()

    def runSpeed(self)-> bool:
        # Dont do anything unless we actually have a step interval
        if self._stepInterval == 0 :
            return False

#        time = micros();
#        if (time - self._lastStepTime >= _stepInterval)
        if self._direction == DIRECTION_CW:
             #Clockwise
            self._current_pos += 1;
        else :
             # Anticlockwise
             self._current_pos -= 1;
        #step(_current_pos);

        #_lastStepTime = time; // Caution: does not account for costs in step()

        return True;
#        else:
#        return false;

    def setSpeed(self, speed: float)->None:
        if speed == self._speed :
            return
#        speed = constrain(speed, -_maxSpeed, _maxSpeed);
        if speed == 0.0 :
            self._stepInterval = 0;
        else :
            self._stepInterval = math.fabs(1000000.0 / speed);
            if speed > 0.0 :
                 self._direction = self.DIRECTION_CW
            else :
                 self._direction = self.DIRECTION_CCW;
        self._speed = speed;

    def acceleration(self) -> float :
        return self._acceleration

    def log(self, s ) ->None :
        if self._printHeader :
            print("s","_n","_step_interval","_speed","_c0","_acceleration","_cn","_cmin","_target_pos","_current_pos")
            self._printHeader = False
        print(s,self._n,self._step_interval, self._speed, self._c0, self._acceleration, self._cn, self._cmin, self._target_pos, self._current_pos)


moresteps = True

stepper = AccelStepper()
step = 0
totalsteps = 2
pattern = 1024

def nextstep() :
  global step,totalsteps,pattern
  step=(++step)%totalsteps;

def loop()-> bool:
    global moresteps, step, stepper, totalsteps
    if moresteps :
        moresteps = False
        if stepper.run():
            moresteps = True # loop again
        return moresteps # loop again

    nextstep();

    distanceToGo = 0.0
    if stepper.current_position == 1024.0 :
        distanceToGo = 0.0
    else :
        distanceToGo = 1024.0
    moresteps = True;
    stepper.move_to(distanceToGo)#reset's speed
    stepper.log("after_next_step")


stepper = AccelStepper()
stepper.setCurrentPosition(0)
stepper.log("setCurrentPosition")
stepper.setAcceleration(100)
stepper.log("setAcceleration")
stepper.setMaxSpeed(100)
stepper.log("setMaxSpeed")
stepper.move(-2048)
stepper.log("move")
while stepper.run() :
    pass
stepper.log("homing")
stepper.setMaxSpeed(1024)
stepper.log("setMaxSpeed")
stepper.setSpeed(1024)
stepper.log("setSpeed")
stepper.setAcceleration(600)
stepper.log("setAcceleration")
stepper.setCurrentPosition(0)
stepper.log("setCurrentPosition")
stepper.run()
stepper.log("prepared")
while loop() :
    stepper.log("loop")
stepper.log("loop")
loop();
while loop() :
    stepper.log("loop2")
stepper.log("loop2")
loop()
while loop() :
    stepper.log("loop3")
stepper.log("loop3")
