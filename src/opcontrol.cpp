#include "main.h"

void opcontrol() {
	pros::lcd::clear_line(5);
	Odom({0, 0, 0});
  Robot::startTask("Odometry", Odom::updatePos);
  Robot::startTask("Display", Robot::displayPosition);
	pros::lcd::print(1, "OPCONTROL");
	Robot::startTask("OPControl", Drive::driverControl);
}
