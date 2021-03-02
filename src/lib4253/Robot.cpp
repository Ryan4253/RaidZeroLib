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

void Drive::resetEncoders() {
	leftEncoder.reset();
	rightEncoder.reset();
  midEncoder.reset();
}

void Drive::resetIMU(){
  imuTop.reset();
  imuBottom.reset();
  pros::delay(2000+100);
}

double Drive::getAngle() {
  return (imuTop.get_rotation() + imuBottom.get_rotation()) / 2;
}

double Drive::getDistance(){
  return (leftEncoder.get_value() + rightEncoder.get_value())/2;
}

Vector Drive::scaleSpeed(double drivePower, double turnPower, double turnScale){
  double leftPower = drivePower - turnPower * turnScale;
  double rightPower = drivePower + turnPower * turnScale;

  double maxPower = fmax(std::fabs(leftPower), std::fabs(rightPower));
  double adjustment = driveSlew.step(maxPower);
  leftPower = leftPower / maxPower * adjustment;
  rightPower = rightPower / maxPower * adjustment;

  return {leftPower, rightPower};
}

void Drive::moveDistance(double dist, QTime timeLimit) {
  Pose currentPos = Odom::getPos();
  Vector target = (currentPos.toVector()).add({currentPos.x * cos(currentPos.angle), currentPos.y * sin(currentPos.angle)});
  moveTo(target, 1, timeLimit);
}

void Drive::moveTo(Vector target, double turnScale, QTime timeLimit){
  drivePID.initialize();
  turnPID.initialize();
  driveSlew.reset();
  Timer timer = Timer();
  double distToTarget; QTime startTime = timer.millis();

  do{
    Pose currentPos = Odom::getPos();
    Vector closestPoint = currentPos.closest(target);
    pros::lcd::print(0, "CURRENT X: %lf", (double)currentPos.x);
    pros::lcd::print(1, "CURRENT Y: %lf", (double)currentPos.y);
    pros::lcd::print(2, "CURRENT A: %lf", (double)currentPos.angle);

    distToTarget = currentPos.distanceTo(target);
    double angleToTarget = currentPos.angleTo(target);
    double distToClose = currentPos.distanceTo(closestPoint);
    double angleToClose = currentPos.angleTo(closestPoint);

    double driveError = (std::fabs(angleToClose) >= 90) ? -distToClose : distToClose;
    double turnError = (std::fabs(distToTarget) < 5)  ? 0 : Math::wrapAngle90(angleToTarget);

    pros::lcd::print(4, "Drive Error: %lf", driveError);
    pros::lcd::print(5, "Turn Error: %lf", turnError);

    double drivePower = Drive::drivePID.update(-driveError);
    double turnPower = Drive::turnPID.update(-turnError);

    pros::lcd::print(6, "Drive Power: %lf", drivePower);
    pros::lcd::print(7, "Turn Power: %lf", turnPower);
    Vector driveSpeed = Drive::scaleSpeed(drivePower, turnPower, 0.3);

    Robot::setPower(baseLeft, driveSpeed.x);
    Robot::setPower(baseRight, driveSpeed.y);
    pros::delay(10);


  }while((distToTarget >= 0.125) && timeLimit > (timer.millis()-startTime));
  Robot::setPower(base, 0);
}

void Drive::turnAngle(double angle, QTime timeLimit){
  Drive::turnPID.initialize();
  driveSlew.reset();
  double initAngle = Odom::getAngleDeg(), error, power;
  angle = Math::wrapAngle180(angle);
  Timer timer = Timer();
  QTime startTime = timer.millis();


  do{
    error = (angle - (Odom::getAngleDeg()-initAngle));
    power = turnPID.update(error);
    power = Drive::driveSlew.step(power);

    Robot::setPower(baseLeft, power);
    Robot::setPower(baseRight, -power);
    pros::delay(10);
  }while((error >= 2) && timeLimit > (timer.millis()-startTime));

  Robot::setPower(base, 0);
}

void Drive::turnToAngle(double angle, QTime timeLimit){
  Drive::turnPID.initialize();
  Drive::driveSlew.reset();
  angle = Math::wrapAngle180(angle); double error, power;
  Timer timer = Timer(); QTime startTime = timer.millis();

  do{
    error = Math::wrapAngle180(angle - (Odom::getAngleDeg()));
    power = turnPID.update(error);
    power = Drive::driveSlew.step(power);

    Robot::setPower(baseLeft, power);
    Robot::setPower(baseRight, -power);
    pros::delay(10);
  }while((error >= 2) && timeLimit > (timer.millis()-startTime));

  Robot::setPower(base, 0);
}

void Drive::driverControl(void* ptr){
  while(true){
  	 //double leftPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
  	 //double rightPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

  	double leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  	double rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  	Robot::setPower(baseLeft, leftPower);
    Robot::setPower(baseRight, rightPower);

    pros::delay(3);
  }
}
