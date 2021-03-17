#include "main.h"

pros::Controller master(CONTROLLER_MASTER);

pros::Motor LF(10, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB(7, pros::E_MOTOR_GEARSET_06, false);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

pros::ADIEncoder leftEncoder('A', 'B', true);
pros::ADIEncoder rightEncoder('E', 'F', false);
pros::ADIEncoder midEncoder('C', 'D', false);
pros::ADIButton leftAutonSelector('G');
pros::ADIButton rightAutonSelector('H');

std::vector<pros::Motor> base = {LF, LB, RF, RB};
std::vector<pros::Motor> baseLeft = {LB, LF};
std::vector<pros::Motor> baseRight = {RB, RF};

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

PID Drive::drivePID;
PID Drive::turnPID;
SlewController Drive::driveSlew(9, 256);

std::shared_ptr<ChassisController> Drive::chassis
  = ChassisControllerBuilder()
    .withMotors({9, 10}, {7, 8})
    .withDimensions({AbstractMotor::gearset::blue, 7.0 / 3.0}, {{4.00_in, 12.25_in}, imev5GreenTPR})
    .build();

void Robot::setPower(std::vector<pros::Motor> motor, double power){
  for(int i = 0; i < motor.size(); i++){
    motor[i] = power;
  }
}

void Robot::setBrakeMode(std::vector<pros::Motor> motor, std::string mode){
  pros::motor_brake_mode_e_t brakeMode;

  if(mode == "coast")
    brakeMode = MOTOR_BRAKE_COAST;
  else if(mode == "brake")
    brakeMode = MOTOR_BRAKE_BRAKE;
  else
    brakeMode = MOTOR_BRAKE_HOLD;

  for(int i = 0; i < motor.size(); i++){
    motor[i].set_brake_mode(brakeMode);
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

void Robot::startTask(std::string name, void (*func)(void *)) {
	if (!taskExists(name)) {
		Robot::tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, (void*)0 ,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
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
