#include "main.h"
#include "declarations.hpp"
using namespace lib4253;
using namespace okapi::literals;

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
	chassis->setState(DriveState::TANK);
	int prevAState = 0;

	while(true){
		switch(chassis->getState()){
			case DriveState::TANK:
				chassis->tank(master.getAnalog(okapi::ControllerAnalog::leftY), master.getAnalog(okapi::ControllerAnalog::rightY));
				break;

			case DriveState::ARCADE:
				chassis->arcade(master.getAnalog(okapi::ControllerAnalog::leftY), master.getAnalog(okapi::ControllerAnalog::rightX));
				break;

			default:
				break;
		}
		
		int aState = master.getDigital(okapi::ControllerDigital::A);
		if(aState && !prevAState){
			if(chassis->getState() == DriveState::TANK){
				chassis->setState(DriveState::ARCADE);
			}
			else{
				chassis->setState(DriveState::ARCADE);
			}
		}

		pros::delay(10);
	}
}
