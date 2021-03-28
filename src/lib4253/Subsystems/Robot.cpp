#include "main.h"

Controller master(ControllerId::master);

//Motor LF(10, pros::E_MOTOR_GEARSET_06, true);
Motor LF(10, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor LB(9, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RF(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor RB(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

ADIEncoder leftEncoder('A', 'B', true);
ADIEncoder rightEncoder('E', 'F', false);
ADIEncoder midEncoder('C', 'D', false);
ADIButton leftAutonSelector('G');
ADIButton rightAutonSelector('H');

std::vector<Motor> base = {LF, LB, RF, RB};
std::vector<Motor> baseLeft = {LB, LF};
std::vector<Motor> baseRight = {RB, RF};

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


/*
std::shared_ptr<ChassisController> Drive::chassis
  = ChassisControllerBuilder()
    .withMotors({9, 10}, {7, 8})
    .withDimensions({AbstractMotor::gearset::blue, 7.0 / 3.0}, {{4.00_in, 12.25_in}, imev5GreenTPR})
    .build();
*/
void Robot::setPower(std::vector<Motor> motor, double power){
  for(int i = 0; i < motor.size(); i++){
    motor[i] = power;
  }
}

void Robot::setBrakeMode(std::vector<Motor> motor, std::string mode){
  AbstractMotor::brakeMode brakeMode;

  if(mode == "coast")
    brakeMode = AbstractMotor::brakeMode::coast;
  else if(mode == "brake")
    brakeMode = AbstractMotor::brakeMode::brake;
  else
    brakeMode = AbstractMotor::brakeMode::hold;

  for(int i = 0; i < motor.size(); i++){
    motor[i].setBrakeMode(brakeMode);
  }
}

void Robot::displayPosition(void* ptr){
  while(true){
    pros::lcd::print(2, "X: %lf", Odom::getX());
    pros::lcd::print(3, "Y: %lf", Odom::getY());
    pros::lcd::print(4, "A: %lf", Odom::getAngleDeg());

    //std::cout << "X: " << Odom::getX() << " Y: " << Odom::getY() << " A: " << Odom::getAngleDeg() << std::endl;
    pros::delay(3);
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
