#include "main.h"

CustomOdometry::CustomOdometry():globalPos(0, 0, 0){
}

Pose CustomOdometry::getPos(){
  return globalPos;
}

double CustomOdometry::getX(){
  return globalPos.x;
}

QLength CustomOdometry::getQX(){
  return globalPos.x * inch;
}

double CustomOdometry::getY(){
  return globalPos.y;
}

QLength CustomOdometry::getQY(){
  return globalPos.y * inch;
}

double CustomOdometry::getAngleDeg(){
  return Math::radToDeg(globalPos.angle);
}

double CustomOdometry::getAngleRad(){
  return globalPos.angle;
}

void CustomOdometry::setPos(Pose newPos){
  globalPos.x = (double)newPos.x;
  globalPos.y = (double)newPos.y;
  globalPos.angle = Math::degToRad(newPos.angle);
}

void CustomOdometry::setX(double x){
  globalPos.x = x;
}

void CustomOdometry::setX(QLength x){
  globalPos.x = x.convert(inch);
}

void CustomOdometry::setY(double y){
  globalPos.y = y;
}

void CustomOdometry::setY(QLength y){
  globalPos.y = y.convert(inch);
}

void CustomOdometry::setAngleDeg(double theta){
  globalPos.angle = Math::radToDeg(theta);
}

void CustomOdometry::setAngleRad(double theta){
  globalPos.angle = theta;
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

ADIThreeWheelOdometry& ADIThreeWheelOdometry::withDimensions(std::tuple<double, double, double> dimension){
  lDist = std::get<0>(dimension);
  mDist = std::get<1>(dimension);
  rDist = std::get<2>(dimension);

  return *this;
}

void ADIThreeWheelOdometry::updatePos(){
  while(true){
    double lVal = leftEncoder.get(), mVal = midEncoder.get(), rVal = rightEncoder.get();

    double left = Math::tickToInch(lVal - lPrev);
    double right = Math::tickToInch(rVal - rPrev);
    double mid = Math::tickToInch(mVal - mPrev);

    lPrev = lVal;
    rPrev = rVal;
    mPrev = mVal;

    double h, h2, rRad, mRad, theta = (left - right) / (LDIST + RDIST);

    if(theta != 0){
      rRad = right / theta;
      mRad = mid / theta;

      h = (rRad + RDIST) * sin(theta / 2) * 2;
      h2 = (mRad + MDIST) * sin(theta / 2) * 2;
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

void ADIThreeWheelOdometry::resetSensors(){
  left.reset(); mid.reset(); right.reset();
}

CustomOdometry* tracker = new ADIThreeWheelOdometry({'A', 'B', true}, {'C', 'D', false}, {'E', 'F', false});
