#include "main.h"

Drive::Drive(const std::initializer_list<Motor> &l, const std::initializer_list<Motor> &r):
 left(l), right(r)
{
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
  return (leftEncoder.get() + rightEncoder.get())/2;
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
  Vector displacement = {dist * cos(currentPos.angle), dist * sin(currentPos.angle)};
  Vector target = currentPos.toVector() + displacement;
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

    Robot::setPower(left, driveSpeed.x);
    Robot::setPower(right, driveSpeed.y);
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

    Robot::setPower(left, power);
    Robot::setPower(right, -power);
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

    Robot::setPower(left, power);
    Robot::setPower(right, -power);
    pros::delay(10);
  }while((error >= 2) && timeLimit > (timer.millis()-startTime));

  Robot::setPower(base, 0);
}

Drive::State Drive::getState(){
  return driveState;
}

void Drive::setState(State s){
  driveState = s;
}

void Drive::updateState(){
  // currently no need
}

void Drive::tank(){
  //double leftPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
  //double rightPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

 double leftPower = master.getAnalog(ControllerAnalog::leftY);
 double rightPower = master.getAnalog(ControllerAnalog::rightY);
 Robot::setPower(left, leftPower);
 Robot::setPower(right, rightPower);

 pros::delay(3);
}

void Drive::arcade(){
  int power = master.getAnalog(ControllerAnalog::leftY);
  int turn = master.getAnalog(ControllerAnalog::rightX);
  int left = power + turn;
  int right = power - turn;
  Robot::setPower(left, Math::clamp(power + turn, -127, 127));
  Robot::setPower(right, Math::clamp(power-turn, -127, 127));

    pros::delay(2);
}

void Drive::execute(){
  switch(driveState){
    case TANK:
      tank();
      break;

    case ARCADE:
      arcade();
      break;

    default:
      break;
  }
}

void Drive::run(){
  while(true){
    updateState();
    execute();
    pros::delay(3);
  }
}

void Drive::driveTask(void* ptr){
	pros::delay(10);
	Drive* that = static_cast<Drive*>(ptr);
	that->run();
}


Drive drive({LF, LB}, {RF, RB});

/*
 = ChassisControllerBuilder()
  .withMotors({9, 10}, {7, 8})
  .withDimensions(AbstractMotor::gearset::green, {{4.32_in, 12.25_in}, imev5GreenTPR})
  .withSensors(ADIEncoder{'A', 'B', true}, ADIEncoder{'C', 'D'})
  .build();
  */
