#pragma once
#include "main.h"
#include "lib4253/Controller/PID.hpp"

namespace lib4253{

class Flywheel{
  std::vector<pros::Motor> flywheel;
  FPID flywheelPID;

  void initialize();
  void setRPM(int target);
};

}
