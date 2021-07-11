#include "Chassis.hpp"
namespace lib4253{
// Constructor
Chassis::Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const okapi::ChassisScales& idimension,
			std::shared_ptr<okapi::IMU> imu):
left(iLeft), right(iRight), dimension(idimension), inertial(imu)
{}


// Task Function
void Chassis::loop(){
    auto t = pros::millis();
    while(true){
        switch(getState()){
            case DriveState::ARCADE:
                setPower(scaleSpeed(lControllerY, rControllerX, 12000));
                break;
            
            case DriveState::TANK:
                setPower(desaturate(lControllerY, rControllerY, 12000));
                break;

            default:
                break;
        }
        
        pros::Task::delay_until(&t, 10);
    }
}

// Initializer
void Chassis::initialize() const{
    for(auto& motor : left){
        motor->tarePosition();
        motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    }

    for(auto& motor : right){
        motor->tarePosition();
        motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    }

    if(inertial){
        inertial->calibrate();
        while(inertial->isCalibrating()){
            pros::delay(10);
        }
    }
}

void Chassis::resetSensor() const{
    for(auto& motor : left){
        motor->tarePosition();
    }

    for(auto& motor : right){
        motor->tarePosition();
    }

    if(inertial){
        inertial->reset();
    }
}

// Getter
okapi::QAngle Chassis::getAngle() const{
    okapi::QAngle angle;
    if(inertial){
        angle = inertial->get() * okapi::degree;
    }
    else{
        angle = Math::angleToYaw((getLeftEncoderReading() - getRightEncoderReading()) * okapi::degree, dimension);
    }

    return Math::angleWrap180(angle);
}

okapi::QLength Chassis::getDistance() const{
    return Math::angleToArcLength(getEncoderReading() * okapi::degree, dimension.wheelDiameter/2);
}

double Chassis::getEncoderReading() const{
    double total = 0;
    for(auto& motor : left){
        total += motor->getPosition();
    }

    for(auto& motor : right){
        total += motor->getPosition();
    }

    return total / (left.size() + right.size());
}

double Chassis::getLeftEncoderReading() const{
    double total = 0;
    for(auto& motor : left){
        total += motor->getPosition();
    }

    return total / (left.size());
}

double Chassis::getRightEncoderReading() const{
    double total = 0;
    for(auto& motor : right){
        total += motor->getPosition();
    }

    return total / (right.size());
}   

okapi::ChassisScales Chassis::getDimension() const{
    return dimension;
}

// Setter
void Chassis::setBrakeType(const okapi::AbstractMotor::brakeMode& iMode) const{
    for(auto& motor : left){
        motor->setBrakeMode(iMode);
    }

    for(auto& motor : right){
        motor->setBrakeMode(iMode);
    }
}

void Chassis::setPower(const double& lPower, const double& rPower) const{
    for(auto& motor : left){
        motor->moveVoltage(lPower);
    }

    for(auto& motor : right){
        motor->moveVoltage(rPower);
    }
}

void Chassis::setPower(const std::pair<double, double>& power) const{
    for(auto& motor : left){
        motor->moveVoltage(power.first);
    }

    for(auto& motor : right){
        motor->moveVoltage(power.second);
    }
}

void Chassis::setVelocity(const double& lVelocity, const double& rVelocity) const{
    for(auto& motor : left){
        motor->moveVelocity(lVelocity);
    }

    for(auto& motor : right){
        motor->moveVelocity(rVelocity);
    }
}

void Chassis::setVelocity(const std::pair<double, double>& velocity) const{
    for(auto& motor : left){
        motor->moveVelocity(velocity.first);
    }

    for(auto& motor : right){
        motor->moveVelocity(velocity.second);
    }
}

void Chassis::setVelocity(const std::pair<okapi::QSpeed, okapi::QAcceleration>& leftKinematics, const std::pair<okapi::QSpeed, okapi::QAcceleration>& rightKinematics) const{
    okapi::QSpeed leftSpeed = left[0]->getFilteredRPM() * M_PI / 30 * dimension.wheelDiameter.convert(okapi::meter)/2 * okapi::mps;
    okapi::QSpeed rightSpeed = right[0]->getFilteredRPM() * M_PI / 30 * dimension.wheelDiameter.convert(okapi::meter)/2 * okapi::mps;

    for(int i = 0; i < left.size(); i++){
        left[i]->setVelocity(leftKinematics.first, leftKinematics.second, leftSpeed);
    }

    for(int i = 0; i < right.size(); i++){
        right[i]->setVelocity(rightKinematics.first, rightKinematics.second, rightSpeed);
    }
}

// Chassis Movement Method
void Chassis::move(const double& lPower, const double& rPower, const okapi::QTime& timeLim) const{
    setPower(lPower, rPower);
    pros::delay(timeLim.convert(okapi::millisecond));
    setPower(0, 0);
}

// Chassis Math Method
std::pair<double, double> Chassis::scaleSpeed(const double& linear, const double& yaw, const double& max) const{
    double left = linear - yaw;
    double right = linear + yaw;
    return desaturate(left, right, max);
}

std::pair<double, double> Chassis::desaturate(const double& left, const double& right, const double& max) const{
    double leftPower = left, rightPower = right, maxPower = fmax(std::fabs(left), std::fabs(right));
    if(maxPower > std::fabs(max)){
        leftPower = left / maxPower * max;
        rightPower = right / maxPower * max;
    }

    return {leftPower, rightPower};
}   

std::pair<okapi::QSpeed, okapi::QSpeed> Chassis::inverseKinematics(okapi::QSpeed velocity, okapi::QAngularSpeed angularVelocity) const{
    double angVel = angularVelocity.convert(okapi::radps);
    okapi::QSpeed diff = angVel * dimension.wheelTrack.convert(okapi::meter) * okapi::mps;
    return {velocity - diff, velocity + diff};
}

std::pair<okapi::QAcceleration, okapi::QAcceleration> Chassis::inverseKinematics(okapi::QAcceleration acceleration, okapi::QAngularAcceleration angularAcceleration) const{
    double angAccel = angularAcceleration.convert(okapi::radps2);
    okapi::QAcceleration diff = angAccel * dimension.wheelTrack.convert(okapi::meter) * okapi::mps2;
    return {acceleration - diff, acceleration + diff};
}

void Chassis::tank(const double& left, const double& right){
    lControllerY = left * 12000;
    rControllerY = right * 12000;
}

void Chassis::arcade(const double& fwd, const double& yaw){
    lControllerY = fwd * 12000;
    rControllerX = yaw * 12000;
}
}
/*
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

Drive& Drive::withPurePursuit(std::tuple<double> lookAhead, std::tuple<double> turnGain, std::tuple<double, double> kinematics){
    PPTenshi.setLookAhead(std::get<0>(lookAhead));
    PPTenshi.setTurnGain(std::get<0>(turnGain));
    PPTenshi.setKinematics(std::get<0>(kinematics), std::get<1>(kinematics));
    return *this;
}

Drive& Drive::withSlew(int acc, int dec){
    driveSlew.setStep(acc, dec);
    return *this;
}

Drive& Drive::withVelocityFeedForward(std::tuple<double, double, double> l, std::tuple<double, double, double> r){
    leftVelController.setGain(std::get<0>(l), std::get<1>(l), std::get<2>(l));
    rightVelController.setGain(std::get<0>(r), std::get<1>(r), std::get<2>(r));
    return *this;
}

void Drive::moveDistanceLMP(double distance){
    okapi::Timer timer;
    bruhMobile->setDistance(distance);
    double initTime = timer.millis().convert(okapi::second), timeElapsed;
    double initAngle = odom->getAngleDeg();
    do {
        timeElapsed = timer.millis().convert(okapi::second) - initTime;
        double velocity = bruhMobile->getVelocityTime(timeElapsed);
        double angle = odom->getAngleDeg() - initAngle;

        double rpm = Math::linearVelToRPM(velocity, gearRatio, wheelSize/2);
        left.moveVelocity(rpm + angle * 5);
        right.moveVelocity(rpm - angle * 5);

    } while(timeElapsed <= bruhMobile->getTotalTime());

    Robot::setPower(left, 0);
    Robot::setPower(right, 0);
}   

void Drive::moveDistanceLMPD(double dist){

    double initLeft = odom->getEncoderLeft();
    double initRight = odom->getEncoderRight();
    double initAngle = odom->getAngleDeg();
    double totalDist;

    do {
        double leftDist = odom->getEncoderLeft() - initLeft;
        double rightDist = odom->getEncoderRight() - initRight;
        totalDist = (leftDist + rightDist) / 2;
        double angle = odom->getAngleDeg() - initAngle;

        double velocity = bruhMobile->getVelocityDist(totalDist);
        double rpm = Math::linearVelToRPM(velocity, gearRatio, wheelSize/2);
        left.moveVelocity(rpm + angle * 5);
        right.moveVelocity(rpm - angle * 5);

    } while(totalDist < dist);
}

void Drive::followPath(SimplePath path){
    PPTenshi.setPath(path);

    do {
        Pose2D currentPos = odom->getPos();
        std::pair<double, double> velocity = PPTenshi.step(currentPos);
        double leftVel = Math::linearVelToRPM(velocity.first, gearRatio, wheelSize/2);
        double rightVel = Math::linearVelToRPM(velocity.second, gearRatio, wheelSize/2);

        left.moveVelocity(leftVel);
        right.moveVelocity(rightVel);
    } while(!PPTenshi.isSettled());

    Robot::setPower(left, 0);
    Robot::setPower(right, 0);
}   

void Drive::followPath(std::string name){
    Trajectory path = Robot::getPath(name);
    int n = path.getSize();
    EmaFilter velFilter(0.65);
    velFilter.reset();

    for(int i = 0; i < n; i++){
        double leftActualVel = velFilter.filter(left.getActualVelocity());
        std::cout << i-1 << " " << leftActualVel << std::endl;
        std::pair<TrajectoryPoint, TrajectoryPoint> kin = path.getKinematics(i);
        double leftRPM = kin.first.velocity * 12 * 60 / (2 * M_PI) / 2 * 7 / 3;
        std::cout << i << " " << leftRPM << " ";

        
        	double rightRPM = kin.second.velocity * 12 * 60 / (2 * M_PI) / 2 * 7 / 3;

        std::cout << i << " " << leftRPM << " ";
        //std::cout << leftRPM << " " << rightRPM << std::endl;

        left.moveVelocity(leftRPM);
        right.moveVelocity(rightRPM);
        
        double l = leftVelController.calcPower(kin.first, left.getActualVelocity());
        double r = rightVelController.calcPower(kin.second, right.getActualVelocity());
        Robot::setPower(left, l);
        Robot::setPower(right, r);

        pros::delay(10);
    }
    left.moveVoltage(0);
    right.moveVoltage(0);

}


void Drive::moveDistance(double dist, okapi::QTime timeLimit) {
    Pose2D currentPos = odom->getPos();
    Point2D displacement = {dist * cos(currentPos.angle), dist * sin(currentPos.angle)};
    Point2D target = currentPos + displacement;
    moveTo(target, 1, timeLimit);
}

void Drive::moveTo(Point2D target, double turnScale, okapi::QTime timeLimit){
    drivePID.initialize();
    turnPID.initialize();
    driveSlew.reset();
    okapi::Timer timer = okapi::Timer();
    double distToTarget; okapi::QTime startTime = timer.millis();

    do {
        Pose2D currentPos = odom->getPos();
        Point2D closestPoint = currentPos.closest(target);
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
        Point2D driveSpeed = Drive::scaleSpeed(drivePower, turnPower, 0.3);

        Robot::setPower(left, driveSpeed.x);
        Robot::setPower(right, driveSpeed.y);
        pros::delay(10);
    } while((distToTarget >= 0.125) && timeLimit > (timer.millis()-startTime));
    Robot::setPower(left, 0);
    Robot::setPower(right, 0);
}

void Drive::turnAngle(double angle, okapi::QTime timeLimit){
    turnPID.initialize();
    driveSlew.reset();
    double initAngle = odom->getAngleDeg(), error, power;
    angle = Math::wrapAngle180(angle);
    okapi::Timer timer = okapi::Timer();
    okapi::QTime startTime = timer.millis();


    do {
        error = (angle - (odom->getAngleDeg()-initAngle));
        power = turnPID.update(error);
        power = driveSlew.step(power);

        Robot::setPower(left, power);
        Robot::setPower(right, -power);
        pros::delay(10);
    } while((error >= 2) && timeLimit > (timer.millis()-startTime));

    Robot::setPower(left, 0);
    Robot::setPower(right, 0);
}

void Drive::turnToAngle(double angle, okapi::QTime timeLimit){
    turnPID.initialize();
    driveSlew.reset();
    angle = Math::wrapAngle180(angle); double error, power;
    okapi::Timer timer = okapi::Timer(); okapi::QTime startTime = timer.millis();

    do {
        error = Math::wrapAngle180(angle - (odom->getAngleDeg()));
        power = turnPID.update(error);
        power = driveSlew.step(power);

        Robot::setPower(left, power);
        Robot::setPower(right, -power);
        pros::delay(10);
    } while((error >= 2) && timeLimit > (timer.millis()-startTime));

    Robot::setPower(left, 0);
    Robot::setPower(right, 0);
}
*/


