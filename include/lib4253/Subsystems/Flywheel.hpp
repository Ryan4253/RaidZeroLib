#pragma once
#include "main.h"

class Flywheel{
  std::vector<pros::Motor> flywheel;
  FPID flywheelPID;

  void initialize();
  void setRPM(int target);
};
