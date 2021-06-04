#include "main.h"
namespace lib4253{

Chassis::Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			std::shared_ptr<ChassisScales> iScale,
			std::shared_ptr<IMU> imu,
			std::unique_ptr<SlewController> _driveSlew,
			std::unique_ptr<PID> _drivePID, 
			std::unique_ptr<PID> _turnPID,
			std::unique_ptr<PID> _anglePID):
left(iLeft), right(iRight)
{
    inertial = imu;
    scale = std::move(iScale);
    driveSlew = std::move(_driveSlew);
    drivePID = std::move(_drivePID);
    turnPID = std::move(_turnPID);
    anglePID = std::move(_anglePID);    
}

Chassis::Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			std::shared_ptr<ChassisScales> iScale,
			std::unique_ptr<SlewController> _driveSlew,
			std::unique_ptr<PID> _drivePID, 
			std::unique_ptr<PID> _turnPID,
			std::unique_ptr<PID> _anglePID,
			std::shared_ptr<IMU> imu):
left(iLeft), right(iRight)
{
    inertial = imu;
    scale = std::move(iScale);
    driveSlew = std::move(_driveSlew);
    drivePID = std::move(_drivePID);
    turnPID = std::move(_turnPID);
    anglePID = std::move(_anglePID);    
}

void Chassis::loop(){
    while(true){
        auto now = pros::millis();
        /*
        switch(currentState.load()){
            case State::TANK:
                tank();
                break;

            case State::ARCADE:
                arcade();
                break;
        }
        */
        pros::Task::delay_until(&now, 10);
    }
}

Chassis::State Chassis::getState(){
    return currentState.load();
}

void Chassis::setState(const Chassis::State& s){
    currentState = s;
}

void Chassis::initialize(){
    for(auto& motor : left){
        motor->tarePosition();
        motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    }

    for(auto& motor : right){
        motor->tarePosition();
        motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    }

    if(driveSlew != nullptr){
        driveSlew->reset();
    }

    if(drivePID != nullptr){
        drivePID->initialize();
    }

    if(turnPID != nullptr){
        turnPID->initialize();
    }

    if(anglePID != nullptr){
        anglePID->initialize();
    }

    if(inertial != nullptr){
        inertial->calibrate();
    }
}

void Chassis::setBrakeType(const AbstractMotor::brakeMode& iMode){
    for(auto& motor : left){
        motor->setBrakeMode(iMode);
    }

    for(auto& motor : right){
        motor->setBrakeMode(iMode);
    }
}

void Chassis::resetSensor(){
    for(auto& motor : left){
        motor->tarePosition();
    }

    for(auto& motor : right){
        motor->tarePosition();
    }

    if(inertial != nullptr){
        inertial->reset();
    }
}

double Chassis::getIMUReading(){
    return inertial->get();
}

double Chassis::getEncoderReading(){
    double total = 0;
    for(auto& motor : left){
        total += motor->getPosition();
    }

    for(auto& motor : right){
        total += motor->getPosition();
    }

    return total / (left.size() + right.size());
}

double Chassis::getLeftEncoderReading(){
    double total = 0;
    for(auto& motor : left){
        total += motor->getPosition();
    }

    return total / (left.size());
}

double Chassis::getRightEncoderReading(){
    double total = 0;
    for(auto& motor : right){
        total += motor->getPosition();
    }

    return total / (right.size());
}   

void Chassis::setPower(const double& lPower, const double& rPower){
    for(auto& motor : left){
        motor->moveVoltage(lPower);
    }

    for(auto& motor : right){
        motor->moveVoltage(rPower);
    }
}

void Chassis::setPower(const std::pair<double, double> power){
    for(auto& motor : left){
        motor->moveVoltage(power.first);
    }

    for(auto& motor : right){
        motor->moveVoltage(power.second);
    }
}

void Chassis::setVelocity(const double& lVelocity, const double& rVelocity){
    for(auto& motor : left){
        motor->setRPM(lVelocity);
    }

    for(auto& motor : right){
        motor->setRPM(rVelocity);
    }
}

void Chassis::setVelocity(const std::pair<double, double> velocity){
    for(auto& motor : left){
        motor->setRPM(velocity.first);
    }

    for(auto& motor : right){
        motor->setRPM(velocity.second);
    }
}

void Chassis::move(const double& lPower, const double& rPower, const QTime& timeLim){
    setPower(lPower, rPower);
    pros::delay(timeLim.convert(millisecond));
    setPower(0, 0);
}

void Chassis::moveDistance(const double& dist, const QTime& timeLim){
    double startTime = pros::millis(), distTravelled, power;

    if(drivePID == nullptr){
        do{
            distTravelled = Math::inchToTick(getEncoderReading(), scale->wheelDiameter.convert(inch), 360);
            power = driveSlew->step(12000);
            if(dist < 0){
                setPower(-power, -power);
            }
            else{
                setPower(power, power);

            }
        }while(std::fabs(distTravelled) < std::fabs(dist) && (pros::millis() - startTime) < timeLim.convert(millisecond));
    }
    else{
        driveSlew->reset();
        drivePID->initialize();
        if(anglePID != nullptr){
            anglePID->initialize();
        }
        resetSensor();

        do{
            double error = dist - Math::tickToInch(getEncoderReading(), scale->wheelDiameter.convert(inch), 360);
            double power = drivePID->update(error), adjustment;

            if(anglePID == nullptr){
                adjustment = 0;
            }
            else if(inertial == nullptr){
                double tickTravelled = getLeftEncoderReading() - getRightEncoderReading();
                adjustment = anglePID->update(Math::wrapAngle180(Math::tickToDeg(tickTravelled, scale, 360)));
            }
            else{
                adjustment = anglePID->update(Math::wrapAngle180(inertial->get()));
            }
            
            std::pair<double, double> finalPower = scaleSpeed(power, adjustment, driveSlew->step(std::fabs(power)));
            setPower(finalPower.first, finalPower.second);
        }while(drivePID->getError() < 0.5 && (pros::millis() - startTime) < timeLim.convert(millisecond));
    }

    setPower(0, 0);
}

void Chassis::turnAngle(const double& angle, const QTime& timeLim){
    double startTime = pros::millis();
    double target = Math::wrapAngle180(angle);
    resetSensor();

    if(turnPID == nullptr){
        double degTravelled;
        do{
            if(inertial != nullptr){
                degTravelled = Math::wrapAngle180(inertial->get());
            }
            else{
                double tickTravelled = getLeftEncoderReading() - getRightEncoderReading();
                degTravelled = Math::wrapAngle180(Math::tickToDeg(tickTravelled, scale, 360));
            }

            if(target > 0){
                setPower(127, -127);
            }
            else{
                setPower(-127, 127);
            }
        }while(std::fabs(degTravelled) < std::fabs(target) && (pros::millis() - startTime) < timeLim.convert(millisecond));
    }
    else{
        double error;
        do{
            if(inertial != nullptr){
                error = target - Math::wrapAngle180(inertial->get());
            }
            else{
                double tickTravelled = getLeftEncoderReading() - getRightEncoderReading();
                error = target - Math::wrapAngle180(Math::tickToDeg(tickTravelled, scale, 360));
            }

            double power = turnPID->update(error);
            setPower(desaturate(power, -power, driveSlew->step(std::abs(power))));
            
        }while(std::fabs(turnPID->getError()) < 0.5 && (pros::millis() - startTime) < timeLim.convert(millisecond));
    }

    setPower(0, 0);
}

std::pair<double, double> Chassis::scaleSpeed(const double& linear, const double& yaw, const double& max){
    double left = linear - yaw;
    double right = linear + yaw;
    return desaturate(left, right, max);
}

std::pair<double, double> Chassis::desaturate(const double& left, const double& right, const double& max){
    double leftPower = left, rightPower = right, maxPower = fmax(std::fabs(left), std::fabs(right));
    if(maxPower > std::fabs(max)){
        leftPower = left / maxPower * max;
        rightPower = right / maxPower * max;
    }

    return {leftPower, rightPower};
}   

void Chassis::tank(const double& left, const double& right){
    //std::cout<<"TANK" << std::endl;.    
    setPower(left * 12000, right * 12000);
}

void Chassis::arcade(const double& fwd, const double& yaw){
    std::pair<double, double> power = scaleSpeed(fwd * 12000, yaw * 12000, 12000);
    setPower(power.first, power.second);
}
}
/*
Drive::Drive(const std::initializer_list<okapi::Motor> &l, const std::initializer_list<okapi::Motor> &r):
    left(l), right(r)
{
}

Drive& Drive::withOdometry(CustomOdometry* tracker){
    odom = tracker;
    return *this;
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