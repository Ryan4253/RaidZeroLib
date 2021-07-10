
#include "Odometry.hpp"
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

Pose2D Odometry::getPos() const{
    return globalPos;
}

okapi::QLength Odometry::getX() const{
    return globalPos.getX();
}

okapi::QLength Odometry::getY() const{
    return globalPos.getY();
}

okapi::QAngle Odometry::getAngle() const{
    return globalPos.getTheta();
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
    globalPos.translation = newPos.translation;
    globalPos.rotation = newPos.rotation;
}

void Odometry::setX(const okapi::QLength& x){
    globalPos.translation.x = x;
}

void Odometry::setY(const okapi::QLength& y){
    globalPos.translation.y = y;
}

void Odometry::setAngle(const okapi::QAngle& theta){
    globalPos.rotation.value = theta;
}

void Odometry::displayPosition() const{
    std::cout 
    << " X: " << globalPos.translation.x.convert(okapi::inch) 
    << " Y: " << globalPos.translation.y.convert(okapi::inch) 
    << " A: " << globalPos.rotation.value.convert(okapi::radian)
    << std::endl;
    pros::lcd::print(2, "X: %lf", globalPos.getX().convert(okapi::inch));
    pros::lcd::print(3, "Y: %lf", globalPos.getY().convert(okapi::inch));
    pros::lcd::print(4, "A: %lf", globalPos.rotation.value.convert(okapi::degree));
} 

void Odometry::resetState(){
    globalPos.translation.x = 0 * okapi::meter;
    globalPos.translation.y = 0 * okapi::meter;
    globalPos.rotation.value = 0 * okapi::radian;
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
    dimension.tpr = okapi::degree;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
}

ThreeWheelOdometry::ThreeWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim){
    left = l;
    mid = m;
    right = r;
    dimension = dim;
    if(dimension.rDist == (-1 * okapi::inch)){
        throw std::invalid_argument("MISSING RIGHT WHEEL OFFSET ARGUMENT");
    }
    dimension.tpr = okapi::rotationDeg;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
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

        okapi::QLength left = Math::angleToArcLength((lVal - lPrev) * dimension.tpr, dimension.wheelDiameter/2);
        okapi::QLength right = Math::angleToArcLength((rVal - rPrev) * dimension.tpr, dimension.wheelDiameter/2);
        okapi::QLength mid = Math::angleToArcLength((mVal - mPrev) * dimension.tpr, dimension.wheelDiameter/2);

        lPrev = lVal;
        rPrev = rVal;
        mPrev = mVal;

        okapi::QLength h, h2, rRad, mRad;
        okapi::QAngle theta = (left - right) / (dimension.lDist + dimension.rDist) * okapi::radian;
        if(theta != 0 * okapi::radian){
            rRad = right / theta.convert(okapi::radian);
            mRad = mid / theta.convert(okapi::radian);

            h = (rRad + dimension.rDist) * sin(theta / 2) * 2;
            h2 = (mRad + dimension.mDist) * sin(theta / 2) * 2;
        }
        else{
            h = right;
            h2 = mid;
        }

        okapi::QAngle endAngle = theta / 2 + globalPos.getTheta();

        globalPos.translation.x += (h * sin(endAngle) + h2 * cos(endAngle));
        globalPos.translation.y += (h * cos(endAngle) + h2 * -sin(endAngle));
        globalPos.rotation.value = Math::angleWrap180((globalPos.rotation.value) + theta);

        pros::Task::delay_until(&t, 3);
    }
}

/* Two Wheel + IMU Odometry */

TwoWheelIMUOdometry::TwoWheelIMUOdometry(const std::shared_ptr<okapi::ADIEncoder>& s, const std::shared_ptr<okapi::ADIEncoder>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim){
    side = s;
    mid = m;
    inertial = imu;
    dimension = dim;
    dimension.tpr = okapi::degree;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
}

TwoWheelIMUOdometry::TwoWheelIMUOdometry(const std::shared_ptr<okapi::RotationSensor>& s, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim){
    side = s;
    mid = m;
    inertial = imu;
    dimension = dim;
    dimension.tpr = okapi::rotationDeg;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
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
        
        okapi::QLength side = Math::angleToArcLength((sVal - sPrev) * dimension.tpr, dimension.wheelDiameter/2);
        okapi::QLength mid = Math::angleToArcLength((mVal - mPrev) * dimension.tpr, dimension.wheelDiameter/2);
        okapi::QAngle theta = (inertial->get() - aPrev) * okapi::degree;
        
        sPrev = sVal;
        mPrev = mVal;
        aPrev = aVal;
        
        okapi::QLength h, h2, sRad, mRad;
        if(theta != 0 * okapi::radian){
            sRad = side / theta.convert(okapi::radian);
            mRad = mid / theta.convert(okapi::radian);

            h = (sRad + dimension.lDist) * sin(theta / 2) * 2;
            h2 = (mRad + dimension.mDist) * sin(theta / 2) * 2;
        }
        else{
            h = side;
            h2 = mid;
        }
        
        okapi::QAngle endAngle = theta / 2 + globalPos.rotation.getVal();
        
        globalPos.translation.x += (h * sin(endAngle) + h2 * cos(endAngle));
        globalPos.translation.y += (h * cos(endAngle) + h2 * -sin(endAngle));
        globalPos.rotation.value = Math::angleWrap180((globalPos.rotation.value) + theta);
        
        pros::Task::delay_until(&t, 3);
    }   
}

/* TWO WHEEL ODOMETRY */

TwoWheelOdometry::TwoWheelOdometry(const std::shared_ptr<okapi::ADIEncoder>& l, const std::shared_ptr<okapi::ADIEncoder>& r, const OdomDimension& dim){
    left = l;
    right = r;
    dimension = dim;
    dimension.tpr = okapi::degree;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
}

TwoWheelOdometry::TwoWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim){
    left = l;
    right = r;
    dimension = dim;
    dimension.tpr = okapi::rotationDeg;
    setPos({0 * okapi::inch, 0 * okapi::inch, 0 * okapi::radian});
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

        okapi::QLength left = Math::angleToArcLength((lVal - lPrev) * dimension.tpr, dimension.wheelDiameter/2);
        okapi::QLength right = Math::angleToArcLength((rVal - rPrev) * dimension.tpr, dimension.wheelDiameter/2);

        lPrev = lVal;
        rPrev = rVal;

        okapi::QLength h, rRad;
        okapi::QAngle theta = (left - right) / (dimension.lDist + dimension.mDist) * okapi::radian;
        if(theta != 0 * okapi::radian){
            rRad = right / theta.convert(okapi::radian);
            h = (rRad + dimension.mDist) * sin(theta / 2) * 2;
        }
        else{
            h = right;
        }

        okapi::QAngle endAngle = theta / 2 + globalPos.getTheta();

        globalPos.translation.x += (h * sin(endAngle));
        globalPos.translation.y += (h * cos(endAngle));
        globalPos.rotation.value = Math::angleWrap180((globalPos.rotation.value) + theta);

        pros::Task::delay_until(&t, 3);
    }
}
}