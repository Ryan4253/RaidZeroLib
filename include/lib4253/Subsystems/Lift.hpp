#pragma once
#include "main.h"
#include "lib4253/Controller/PID.hpp"

namespace lib4253{

class Lift{
  std::vector<pros::Motor> liftMotor;
  PID liftPID;
  void initialize();
};

}
