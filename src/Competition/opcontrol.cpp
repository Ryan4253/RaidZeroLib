#include "main.h"

void opcontrol() {
	int x;
	pros::lcd::clear_line(5);
	pros::lcd::print(1, "OPCONTROL");
	OdomController::setPos({0, 0, 0});
	//drive.setState(Drive::TANK);
	//Drive drive({LF, LB}, {RF, RB});
  Robot::startTask("Odometry", OdomController::updatePos, &x);
  Robot::startTask("Display", Robot::displayPosition, &x);
	Robot::startTask("OPControl", Drive::driveTask, &drive);

	//Robot::endTask("Odometry");
}
