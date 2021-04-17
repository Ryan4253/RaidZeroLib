#include "main.h"
#include "declarations.hpp"

void opcontrol() {
	int x;
	pros::lcd::clear_line(5);
	pros::lcd::print(1, "OPCONTROL");
	tracker->setPos({0, 0, 0});

  Robot::startTask("Odometry", CustomOdometry::odomTask, tracker);
  Robot::startTask("Display", Robot::displayPosition, &x);
	Robot::startTask("OPControl", Drive::driveTask, &drive);



	//Robot::endTask("Odometry");
}
