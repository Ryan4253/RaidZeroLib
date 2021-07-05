
#include "lib4253/Chassis/Odometry.hpp"
namespace lib4253{

/* Odom Dimesions */

OdomDimension::OdomDimension(const okapi::QLength& wheelDiam, const okapi::QLength& leftOffset, const okapi::QLength& midOffset, const okapi::QLength& rightOffset){
    wheelDiameter = wheelDiam;
    lDist = leftOffset;
    mDist = midOffset;
    rDist = rightOffset;
}

OdomDimension::OdomDimension(const okapi::QLength& wheelDiam, const okapi::QLength& offset1, const okapi::QLength& offset2){
    wheelDiameter = wheelDiam;
    lDist = offset1;
    mDist = offset2;
}

/* Odometry Base Class */

OdomDimension Odometry::withDimension(const okapi::QLength& wheelDiam, const okapi::QLength& leftOffset, const okapi::QLength& midOffset, const okapi::QLength& rightOffset){
    OdomDimension ret(wheelDiam, leftOffset, midOffset, rightOffset);
    return ret;
}

OdomDimension Odometry::withDimension(const okapi::QLength& wheelDiam, const okapi::QLength& offset1, const okapi::QLength& offset2){
    OdomDimension ret(wheelDiam, offset1, offset2);
    return ret;
}

Pose2D Odometry::getPos() const{
    return globalPos;
}

double Odometry::getX() const{
    return globalPos.x;
}

okapi::QLength Odometry::getQX() const{
    return globalPos.x * okapi::inch;
}

double Odometry::getY() const{
    return globalPos.y;
}

okapi::QLength Odometry::getQY() const{
    return globalPos.y * okapi::inch;
}

double Odometry::getAngleDeg() const{
    return Math::radToDeg(globalPos.theta);
}

double Odometry::getAngleRad() const{
    return globalPos.theta;
}

double Odometry::getEncoderLeft() const{
    std::cout << "THIS METHOD HAVE NOT BEEN OVERWRITTEN" << std::endl;
    return -1;
}

double Odometry::getEncoderMid() const{
    std::cout << "THIS METHOD HAVE NOT BEEN OVERWRITTEN" << std::endl;
    return -1;
}

double Odometry::getEncoderRight() const{
    std::cout << "THIS METHOD HAVE NOT BEEN OVERWRITTEN" << std::endl;
    return -1;
}

double Odometry::getEncoderSide() const{
    std::cout << "THIS METHOD HAVE NOT BEEN OVERWRITTEN" << std::endl;
    return -1;
}

void Odometry::setPos(const Pose2D& newPos){
    globalPos.x = (double)newPos.x;
    globalPos.y = (double)newPos.y;
    globalPos.theta = Math::degToRad(newPos.theta);
}

void Odometry::setX(const double& x){
    globalPos.x = x;
}

void Odometry::setX(const okapi::QLength& x){
    globalPos.x = x.convert(okapi::inch);
}

void Odometry::setY(const double& y){
    globalPos.y = y;
}

void Odometry::setY(const okapi::QLength& y){
    globalPos.y = y.convert(okapi::inch);
}

void Odometry::setAngleDeg(const double& theta){
    globalPos.theta = Math::radToDeg(theta);
}

void Odometry::setAngleRad(const double& theta){
    globalPos.theta = theta;
}

void Odometry::displayPosition() const{
    std::cout << "X: " << globalPos.x << " Y: " << globalPos.y << " A: " << globalPos.theta;
    pros::lcd::print(2, "X: %lf", globalPos.x);
    pros::lcd::print(3, "Y: %lf", globalPos.y);
    pros::lcd::print(4, "A: %lf", Math::radToDeg(globalPos.theta));
}

void Odometry::resetState(){
    globalPos.x = 0;
    globalPos.y = 0;
    globalPos.theta = 0;
}

void Odometry::reset(){
    resetState();
    resetSensors();
}

/* Three Wheel Odometry */

ThreeWheelOdometry::ThreeWheelOdometry(const std::shared_ptr<okapi::ADIEncoder>& l, const std::shared_ptr<okapi::ADIEncoder>& m, const std::shared_ptr<okapi::ADIEncoder>& r, const OdomDimension& dim){
    left = l;
    mid = m;
    right = r;
    dimension = dim;
    if(dimension.rDist == (-1 * okapi::inch)){
        throw std::invalid_argument("MISSING RIGHT WHEEL OFFSET ARGUMENT");
    }
    dimension.tpr = 360;
    setPos({0, 0, 0});
}

ThreeWheelOdometry::ThreeWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim){
    left = l;
    mid = m;
    right = r;
    dimension = dim;
    if(dimension.rDist == (-1 * okapi::inch)){
        throw std::invalid_argument("MISSING RIGHT WHEEL OFFSET ARGUMENT");
    }
    dimension.tpr = 4096;
    setPos({0, 0, 0});
}

void ThreeWheelOdometry::resetSensors(){
    left->reset(); mid->reset(); right->reset();
}

double ThreeWheelOdometry::getEncoderLeft() const{
    return lVal;
}

double ThreeWheelOdometry::getEncoderMid() const{
    return mVal;
}

double ThreeWheelOdometry::getEncoderRight() const{
    return rVal;
}

void ThreeWheelOdometry::loop(){
    auto t = pros::millis();
    while(true){
        lVal = left->get(), mVal = mid->get(), rVal = right->get();

        double left = Math::tickToInch(lVal - lPrev, dimension.wheelDiameter.convert(okapi::inch), dimension.tpr);
        double right = Math::tickToInch(rVal - rPrev, dimension.wheelDiameter.convert(okapi::inch), dimension.tpr);
        double mid = Math::tickToInch(mVal - mPrev, dimension.wheelDiameter.convert(okapi::inch), dimension.tpr);

        lPrev = lVal;
        rPrev = rVal;
        mPrev = mVal;

        double h, h2, rRad, mRad, theta = (left - right) / (dimension.lDist.convert(okapi::inch) + dimension.rDist.convert(okapi::inch));
        if(theta != 0){
            rRad = right / theta;
            mRad = mid / theta;

            h = (rRad + dimension.rDist.convert(okapi::inch)) * sin(theta / 2) * 2;
            h2 = (mRad + dimension.mDist.convert(okapi::inch)) * sin(theta / 2) * 2;
        }
        else{
            h = right;
            h2 = mid;
        }

        double endAngle = theta / 2 + globalPos.theta;

        globalPos.x = (double)(globalPos.x) + (h * sin(endAngle) + h2 * cos(endAngle));
        globalPos.y = (double)(globalPos.y) + (h * cos(endAngle) + h2 * -sin(endAngle));
        globalPos.theta = Math::wrapAngle180((double)(globalPos.theta) + theta);

        pros::Task::delay_until(&t, 3);
    }
}

/* Two Wheel + IMU Odometry */

TwoWheelIMUOdometry::TwoWheelIMUOdometry(const std::shared_ptr<okapi::ADIEncoder>& s, const std::shared_ptr<okapi::ADIEncoder>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim){
    side = s;
    mid = m;
    inertial = imu;
    dimension = dim;
    dimension.tpr = 360;
    setPos({0, 0, 0});
}

TwoWheelIMUOdometry::TwoWheelIMUOdometry(const std::shared_ptr<okapi::RotationSensor>& s, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim){
    side = s;
    mid = m;
    inertial = imu;
    dimension = dim;
    dimension.tpr = 4096;
    setPos({0, 0, 0});
}

void TwoWheelIMUOdometry::resetSensors(){
    mid->reset();
    side->reset();
    inertial->reset();
}

double TwoWheelIMUOdometry::getEncoderSide() const{
    return sVal;
}

double TwoWheelIMUOdometry::getEncoderMid() const{
    return mVal;
}

void TwoWheelIMUOdometry::loop(){
    auto t = pros::millis();
    while(true){
        // ldist is sdist
        sVal = side->get(), mVal = mid->get(), aVal = inertial->get();
        
        double side = Math::tickToInch(sVal - sPrev);
        double mid = Math::tickToInch(mVal - mPrev);
        double theta = inertial->get() - aPrev;
        
        sPrev = sVal;
        mPrev = mVal;
        aPrev = aVal;
        
        double h, h2, sRad, mRad;
        if(theta != 0){
            sRad = side / theta;
            mRad = mid / theta;

            h = (sRad + dimension.lDist.convert(okapi::inch)) * sin(theta / 2) * 2;
            h2 = (mRad + dimension.mDist.convert(okapi::inch)) * sin(theta / 2) * 2;
        }
        else{
            h = side;
            h2 = mid;
        }
        
        double endAngle = theta / 2 + globalPos.theta;
        
        globalPos.x = (double)(globalPos.x) + (h * sin(endAngle) + h2 * cos(endAngle));
        globalPos.y = (double)(globalPos.y) + (h * cos(endAngle) + h2 * -sin(endAngle));
        globalPos.theta = Math::wrapAngle180((double)(globalPos.theta) + theta);
        
        pros::Task::delay_until(&t, 3);
    }   
}

/* TWO WHEEL ODOMETRY */

TwoWheelOdometry::TwoWheelOdometry(const std::shared_ptr<okapi::ADIEncoder>& l, const std::shared_ptr<okapi::ADIEncoder>& r, const OdomDimension& dim){
    left = l;
    right = r;
    dimension = dim;
    dimension.tpr = 360;
    setPos({0, 0, 0});
}

TwoWheelOdometry::TwoWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim){
    left = l;
    right = r;
    dimension = dim;
    dimension.tpr = 360;
    setPos({0, 0, 0});
}

void TwoWheelOdometry::resetSensors(){
    left->reset();
    right->reset();
}

double TwoWheelOdometry::getEncoderLeft() const{
    return lVal;
}

double TwoWheelOdometry::getEncoderRight() const{
    return rVal;
}

void TwoWheelOdometry::loop(){
    auto t = pros::millis();
    while(true){
        //mdist is rdist
        lVal = left->get(), rVal = right->get();

        double left = Math::tickToInch(lVal - lPrev, dimension.wheelDiameter.convert(okapi::inch), dimension.tpr);
        double right = Math::tickToInch(rVal - rPrev, dimension.wheelDiameter.convert(okapi::inch), dimension.tpr);

        lPrev = lVal;
        rPrev = rVal;

        double h, rRad, theta = (left - right) / (dimension.lDist.convert(okapi::inch) + dimension.mDist.convert(okapi::inch));
        if(theta != 0){
            rRad = right / theta;

            h = (rRad + dimension.mDist.convert(okapi::inch)) * sin(theta / 2) * 2;
        }
        else{
            h = right;
        }

        double endAngle = theta / 2 + globalPos.theta;

        globalPos.x = (double)(globalPos.x) + (h * sin(endAngle));
        globalPos.y = (double)(globalPos.y) + (h * cos(endAngle));
        globalPos.theta = Math::wrapAngle180((double)(globalPos.theta) + theta);

        pros::Task::delay_until(&t, 3);
    }
}
}