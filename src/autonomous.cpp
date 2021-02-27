#include "main.h"

void autonomous() {
  Odom({0, 0, 90});
  Robot::startTask("Odometry", Odom::updatePos);
  //Robot::startTask("Display", Robot::displayPosition);
  pros::delay(100);
  Drive::moveTo({0, 48}, 1, 200_s);
}
