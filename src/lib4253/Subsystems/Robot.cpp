#include "main.h"
#include "Robot.hpp"

okapi::Controller master(okapi::ControllerId::master);

okapi::Motor LF(10, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor LB(9, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor RF(8, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor RB(7, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

okapi::MotorGroup baseLeft({LF, LB});
okapi::MotorGroup baseRight({RF, RB});
okapi::MotorGroup base({LF, LB, RF, RB});

okapi::ADIEncoder leftEncoder('A', 'B', true);
okapi::ADIEncoder rightEncoder('E', 'F', false);
okapi::ADIEncoder midEncoder('C', 'D', false);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

okapi::ADIButton leftAutonSelector('G');
okapi::ADIButton rightAutonSelector('H');


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;
std::map<std::string, Trajectory> Robot::paths;

void Robot::setPower(okapi::MotorGroup motor, double power){
    motor.moveVoltage(power / 127 * 12000);
}

void Robot::setPower(okapi::Motor motor, double power){
    motor = power;
}

void Robot::setBrakeMode(okapi::MotorGroup motor, brakeType mode){
    okapi::AbstractMotor::brakeMode brakeMode;

    switch(mode){
        case COAST:
        brakeMode = okapi::AbstractMotor::brakeMode::coast;
        break;

        case BRAKE:
        brakeMode = okapi::AbstractMotor::brakeMode::brake;
        break;

        case HOLD:
        brakeMode = okapi::AbstractMotor::brakeMode::hold;
        break;
    }

    motor.setBrakeMode(brakeMode);
}

void Robot::setBrakeMode(okapi::Motor motor, brakeType mode){
    okapi::AbstractMotor::brakeMode brakeMode;

    switch(mode){
        case COAST:
        brakeMode = okapi::AbstractMotor::brakeMode::coast;
        break;

        case BRAKE:
        brakeMode = okapi::AbstractMotor::brakeMode::brake;
        break;

        case HOLD:
        brakeMode = okapi::AbstractMotor::brakeMode::hold;
        break;
    }
    motor.setBrakeMode(brakeMode);
}

void Robot::startTask(std::string name, void (*func)(void *), void *param) {
	if (!taskExists(name)) {
		Robot::tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, param ,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
	}
}

bool Robot::taskExists(std::string name) {
	return tasks.find(name) != tasks.end();
}

void Robot::endTask(std::string name) {
	if (taskExists(name)) {
		tasks.erase(name);
	}
}

void Robot::addPath(std::string name, Trajectory path){
    Robot::paths.insert(std::pair<std::string, Trajectory>(name, path));
}

Trajectory Robot::getPath(std::string name){
    return paths[name];
}

void Robot::deletePath(std::string name){
    paths.erase(name);
}


competition matchState;
