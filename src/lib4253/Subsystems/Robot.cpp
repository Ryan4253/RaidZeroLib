#include "main.h"
#include "Robot.hpp"

Controller master(ControllerId::master);

Motor LF(10, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor LB(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RF(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RB(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup baseLeft({LF, LB});
MotorGroup baseRight({RF, RB});
MotorGroup base({LF, LB, RF, RB});

ADIEncoder leftEncoder('A', 'B', true);
ADIEncoder rightEncoder('E', 'F', false);
ADIEncoder midEncoder('C', 'D', false);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

ADIButton leftAutonSelector('G');
ADIButton rightAutonSelector('H');

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;
std::map<std::string, Trajectory> Robot::paths;

void Robot::setPower(MotorGroup motor, double power){
  motor.moveVoltage(power / 127 * 12000);
}

void Robot::setPower(Motor motor, double power){
  motor = power;
}

void Robot::setBrakeMode(MotorGroup motor, brakeType mode){
  AbstractMotor::brakeMode brakeMode;

  switch(mode){
    case COAST:
      brakeMode = AbstractMotor::brakeMode::coast;
      break;
    case BRAKE:
      brakeMode = AbstractMotor::brakeMode::brake;
      break;
    case HOLD:
      brakeMode = AbstractMotor::brakeMode::hold;
      break;
  }

  motor.setBrakeMode(brakeMode);
}

void Robot::setBrakeMode(Motor motor, brakeType mode){
  AbstractMotor::brakeMode brakeMode;

  switch(mode){
    case COAST:
      brakeMode = AbstractMotor::brakeMode::coast;
      break;
    case BRAKE:
      brakeMode = AbstractMotor::brakeMode::brake;
      break;
    case HOLD:
      brakeMode = AbstractMotor::brakeMode::hold;
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
