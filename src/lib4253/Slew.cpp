#include "main.h"

// Constructor, takes in acceleration and decelaration steps
SlewController::SlewController(int accel, int decel){
  accStep = accel, decStep = decel, speed = 0;
}

// To put simply, slew sets a threshold on how fast your motors can go.
// The steps increase / decreasing over time, making your robot able to accelerate smoothly
void SlewController::reset(){
  speed = 0;
}

int SlewController::step(double target) {
  int step;

  if(std::abs(speed) < std::abs(target)) {
    step = accStep;
  }
  else {
    step = decStep;
  }

  if(target > speed + step) {
    speed += step;
  }
  else if(target < speed - step) {
    speed -= step;
  }
  else {
    speed = target;
  }

  //return fmin(speed, 127);
  return fmin(target, 127);
}
