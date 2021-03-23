#include "main.h"

void opcontrol() {
	int x;
	pros::lcd::clear_line(5);
	Odom({0, 0, 0});
  Robot::startTask("Odometry", Odom::updatePos, &x);
  Robot::startTask("Display", Robot::displayPosition, &x);
	pros::lcd::print(1, "OPCONTROL");
	Robot::startTask("OPControl", Drive::taskFnc, &drive);

	Robot::endTask("Odometry");
}
