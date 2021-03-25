#include "main.h"

class Lift{
  std::vector<pros::Motor> liftMotor;
  PID liftPID;
  void initialize();
};
