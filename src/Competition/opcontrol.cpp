#include "main.h"
#include "lib4253/Utility/declarations.hpp"


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::lcd::clear_line(5); pros::lcd::print(1, "OPCONTROL");
	matchState = OPCONTROL;
	Robot::startTask("Drive", Drive::driveTask, &drive);

	/*
	tracker->setPos({0, 0, 0});
  Robot::startTask("Odometry", CustomOdometry::odomTask, tracker);
	Robot::startTask("OPControl", Drive::driveTask, &drive);
	*/


	//Robot::endTask("Odometry");
}
