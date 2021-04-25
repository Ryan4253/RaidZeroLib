#include "main.h"
#include "lib4253/Utility/auton.hpp"
#include "lib4253/Utility/declarations.hpp"


void autonomous() {
  matchState = AUTONOMOUS;
  pros::lcd::print(0, "auton");
  drive.followPath("8");
}
