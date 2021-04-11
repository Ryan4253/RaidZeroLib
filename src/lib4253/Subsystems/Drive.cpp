#include "main.h"

/* CONSTRUCTOR */

Drive::Drive(const std::initializer_list<Motor> &l, const std::initializer_list<Motor> &r):
 left(l), right(r)
{
}

Drive& Drive::withDrivePID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain){
  drivePID.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
  drivePID.setIGain(std::get<0>(IGain), std::get<1>(IGain));
  drivePID.setEMAGain(std::get<0>(emaGain));
  return *this;
}

Drive& Drive::withTurnPID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain){
  turnPID.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
  turnPID.setIGain(std::get<0>(IGain), std::get<1>(IGain));
  turnPID.setEMAGain(std::get<0>(emaGain));
  return *this;
}

Drive& Drive::withPurePursuit(std::tuple<double, double, double> gain, std::tuple<double> turnGain, std::tuple<double, double> kinematics){
  PPTenshi.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
  PPTenshi.setTurnGain(std::get<0>(turnGain));
  PPTenshi.setKinematics(std::get<0>(kinematics), std::get<1>(kinematics));
  return *this;
}

Drive& Drive::withSlew(int acc, int dec){
  driveSlew.setStep(acc, dec);
  return *this;
}

/* SENSOR FUNCTION */

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

/* STATE FUNCTION */

Drive::State Drive::getState(){
  return driveState;
}

void Drive::setState(State s){
  driveState = s;
}

void Drive::updateState(){
  // currently no need
}

/* TASK FUNCTION */

void Drive::tank(){
  //double leftPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
  //double rightPower = Math::cubicControl(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

  while(true){
    std::cout<<"CHICKEN" << std::endl;
    double leftPower = master.getAnalog(ControllerAnalog::leftY)*127;
    double rightPower = master.getAnalog(ControllerAnalog::rightY)*127;

    Robot::setPower(left, leftPower);
    Robot::setPower(right, rightPower);

    pros::delay(3);
  }

}

void Drive::arcade(){
  int power = master.getAnalog(ControllerAnalog::leftY);
  int turn = master.getAnalog(ControllerAnalog::rightX);
  Robot::setPower(left, Math::clamp(power + turn, -127, 127));
  Robot::setPower(right, Math::clamp(power-turn, -127, 127));

  pros::delay(3);
}

void Drive::run(){
  while(true){
    updateState();

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

    pros::delay(3);
  }
}

void Drive::driveTask(void* ptr){

  Drive* that = static_cast<Drive*>(ptr);
  that->run();
}

/* AUTON FUNCTIONS */

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
  Pose currentPos = OdomController::getPos();
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
    Pose currentPos = OdomController::getPos();
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

    double drivePower = drivePID.update(-driveError);
    double turnPower = turnPID.update(-turnError);

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
  turnPID.initialize();
  driveSlew.reset();
  double initAngle = OdomController::getAngleDeg(), error, power;
  angle = Math::wrapAngle180(angle);
  Timer timer = Timer();
  QTime startTime = timer.millis();


  do{
    error = (angle - (OdomController::getAngleDeg()-initAngle));
    power = turnPID.update(error);
    power = driveSlew.step(power);

    Robot::setPower(left, power);
    Robot::setPower(right, -power);
    pros::delay(10);
  }while((error >= 2) && timeLimit > (timer.millis()-startTime));

  Robot::setPower(base, 0);
}

void Drive::turnToAngle(double angle, QTime timeLimit){
  turnPID.initialize();
  driveSlew.reset();
  angle = Math::wrapAngle180(angle); double error, power;
  Timer timer = Timer(); QTime startTime = timer.millis();

  do{
    error = Math::wrapAngle180(angle - (OdomController::getAngleDeg()));
    power = turnPID.update(error);
    power = driveSlew.step(power);

    Robot::setPower(left, power);
    Robot::setPower(right, -power);
    pros::delay(10);
  }while((error >= 2) && timeLimit > (timer.millis()-startTime));

  Robot::setPower(base, 0);
}

/*
 = ChassisControllerBuilder()
  .withMotors({9, 10}, {7, 8})
  .withDimensions(AbstractMotor::gearset::green, {{4.32_in, 12.25_in}, imev5GreenTPR})
  .withSensors(ADIEncoder{'A', 'B', true}, ADIEncoder{'C', 'D'})
  .build();
  */

Drive drive({-10, 9}, {8, -7});