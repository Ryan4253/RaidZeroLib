#pragma once
#include "main.h"
#include "lib4253/Controller/PID.hpp"

class Lift{
  std::vector<pros::Motor> liftMotor;
  PID liftPID;
  void initialize();
};
