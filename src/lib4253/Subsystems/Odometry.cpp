#include "main.h"
#include "Odometry.hpp"

namespace lib4253{

CustomOdometry::CustomOdometry():globalPos(0, 0, 0){
}

void CustomOdometry::withDimensions(std::tuple<double, double, double> dimensions){}

void CustomOdometry::withDimensions(std::tuple<double, double> dimensions){}

Pose2D CustomOdometry::getPos(){
  return globalPos;
}

double CustomOdometry::getX(){
  return globalPos.x;
}

okapi::QLength CustomOdometry::getQX(){
  return globalPos.x * okapi::inch;
}

double CustomOdometry::getY(){
  return globalPos.y;
}

okapi::QLength CustomOdometry::getQY(){
  return globalPos.y * okapi::inch;
}

double CustomOdometry::getAngleDeg(){
  return Math::radToDeg(globalPos.angle);
}

double CustomOdometry::getAngleRad(){
  return globalPos.angle;
}

double CustomOdometry::getEncoderLeft(){}
double CustomOdometry::getEncoderMid(){}
double CustomOdometry::getEncoderRight(){}

void CustomOdometry::setPos(Pose2D newPos){
  globalPos.x = (double)newPos.x;
  globalPos.y = (double)newPos.y;
  globalPos.angle = Math::degToRad(newPos.angle);
}

void CustomOdometry::setX(double x){
  globalPos.x = x;
}

void CustomOdometry::setX(okapi::QLength x){
  globalPos.x = x.convert(okapi::inch);
}

void CustomOdometry::setY(double y){
  globalPos.y = y;
}

void CustomOdometry::setY(okapi::QLength y){
  globalPos.y = y.convert(okapi::inch);
}

void CustomOdometry::setAngleDeg(double theta){
  globalPos.angle = Math::radToDeg(theta);
}

void CustomOdometry::setAngleRad(double theta){
  globalPos.angle = theta;
}

void CustomOdometry::displayPosition(){
  std::cout << "X: " << globalPos.x << " Y: " << globalPos.y << " A: " << globalPos.angle;
  pros::lcd::print(2, "X: %lf", globalPos.x);
  pros::lcd::print(3, "Y: %lf", globalPos.y);
  pros::lcd::print(4, "A: %lf", Math::radToDeg(globalPos.angle));
}

void CustomOdometry::resetState(){
  globalPos.x = 0;
  globalPos.y = 0;
  globalPos.angle = 0;
}

void CustomOdometry::reset(){
  resetState();
  resetSensors();
}

void CustomOdometry::odomTask(void* ptr){
  pros::delay(10);
  CustomOdometry* that = static_cast<CustomOdometry*>(ptr);
  that->updatePos();
}

/* THREE WHEEL ADI ENCODER ODOMETRY */

ADIThreeWheelOdometry::ADIThreeWheelOdometry(std::tuple<char, char, bool> l, std::tuple<char, char, bool> m, std::tuple<char, char, bool> r)
:left(std::get<0>(l), std::get<1>(l), std::get<2>(l)), mid(std::get<0>(m), std::get<1>(m), std::get<2>(m)), right(std::get<0>(r), std::get<1>(r), std::get<2>(r))
{
  resetSensors();
}

void ADIThreeWheelOdometry::withDimensions(std::tuple<double, double, double> dimension){
  lDist = std::get<0>(dimension);
  mDist = std::get<1>(dimension);
  rDist = std::get<2>(dimension);
}

void ADIThreeWheelOdometry::resetSensors(){
  left.reset(); mid.reset(); right.reset();
}

double ADIThreeWheelOdometry::getEncoderLeft(){
  return lVal;
}

double ADIThreeWheelOdometry::getEncoderMid(){
  return mVal;
}

double ADIThreeWheelOdometry::getEncoderRight(){
  return rVal;
}

void ADIThreeWheelOdometry::updatePos(){
  while(true){
    lVal = left.get(), mVal = mid.get(), rVal = right.get();

    double left = Math::tickToInch(lVal - lPrev);
    double right = Math::tickToInch(rVal - rPrev);
    double mid = Math::tickToInch(mVal - mPrev);

    lPrev = lVal;
    rPrev = rVal;
    mPrev = mVal;

    double h, h2, rRad, mRad, theta = (left - right) / (lDist + rDist);
    if(theta != 0){
      rRad = right / theta;
      mRad = mid / theta;

      h = (rRad + rDist) * sin(theta / 2) * 2;
      h2 = (mRad + mDist) * sin(theta / 2) * 2;
    }
    else{
      h = right;
      h2 = mid;
    }

    double endAngle = theta / 2 + globalPos.angle;

    globalPos.x = (double)(globalPos.x) + (h * sin(endAngle) + h2 * cos(endAngle));
    globalPos.y = (double)(globalPos.y) + (h * cos(endAngle) + h2 * -sin(endAngle));
    globalPos.angle = Math::wrapAngle180((double)(globalPos.angle) + theta);

    pros::delay(3);
  }
}

/* TWO WHEEL ADI ENCODER + IMU ODOMETRY */

ADITwoWheelIMUOdometry::ADITwoWheelIMUOdometry(std::tuple<char, char, bool> s, std::tuple<char, char, bool> m, int port):
  side(std::get<0>(s), std::get<1>(s), std::get<2>(s)), mid(std::get<0>(m), std::get<1>(m), std::get<2>(m)), imu(port)
{}

void ADITwoWheelIMUOdometry::withDimensions(std::tuple<double, double> dimension){
  sDist = std::get<0>(dimension);
  mDist = std::get<1>(dimension);
}

void ADITwoWheelIMUOdometry::resetSensors(){
  mid.reset();
  side.reset();
  imu.reset();
}

double ADITwoWheelIMUOdometry::getEncoderLeft(){
  return sVal;
}

double ADITwoWheelIMUOdometry::getEncoderMid(){
  return mVal;
}

void ADITwoWheelIMUOdometry::updatePos(){
  while(true){
    sVal = side.get(), mVal = mid.get(), aVal = imu.get();

    double side = Math::tickToInch(sVal - sPrev);
    double mid = Math::tickToInch(mVal - mPrev);
    double theta = imu.get() - aPrev;

    sPrev = sVal;
    mPrev = mVal;
    aPrev = aVal;

    double h, h2, sRad, mRad;
    if(theta != 0){
      sRad = side / theta;
      mRad = mid / theta;

      h = (sRad + sDist) * sin(theta / 2) * 2;
      h2 = (mRad + mDist) * sin(theta / 2) * 2;
    }
    else{
      h = side;
      h2 = mid;
    }

    double endAngle = theta / 2 + globalPos.angle;

    globalPos.x = (double)(globalPos.x) + (h * sin(endAngle) + h2 * cos(endAngle));
    globalPos.y = (double)(globalPos.y) + (h * cos(endAngle) + h2 * -sin(endAngle));
    globalPos.angle = Math::wrapAngle180((double)(globalPos.angle) + theta);

    pros::delay(3);
  }
}

}
