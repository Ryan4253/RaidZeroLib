#include "main.h"

Controller master(ControllerId::master);

//Motor LF(10, pros::E_MOTOR_GEARSET_06, true);
Motor LF(10, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor LB(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RF(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RB(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

ADIEncoder leftEncoder('A', 'B', true);
ADIEncoder rightEncoder('E', 'F', false);
ADIEncoder midEncoder('C', 'D', false);
ADIButton leftAutonSelector('G');
ADIButton rightAutonSelector('H');

MotorGroup baseLeft({LF, LB});
MotorGroup baseRight({RF, RB});
MotorGroup base({LF, LB, RF, RB});

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;





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

void Robot::displayPosition(void* ptr){
  while(true){
    pros::lcd::print(2, "X: %lf", OdomController::getX());
    pros::lcd::print(3, "Y: %lf", OdomController::getY());
    pros::lcd::print(4, "A: %lf", OdomController::getAngleDeg());

    //std::cout << "X: " << Odom::getX() << " Y: " << Odom::getY() << " A: " << Odom::getAngleDeg() << std::endl;
    pros::delay(3);
    //std::cout << "OHBOYITSALLCOMINGTOGETHER" << std::endl;
  }
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
