#include "main.h"

Pose Odom::globalPos = {0, 0, 0};
double Odom::lPrev = 0, Odom::rPrev = 0, Odom::mPrev = 0;

Odom::Odom(Pose initial){
  globalPos.x = (double)initial.x;
  globalPos.y = (double)initial.y;
  globalPos.angle = Math::degToRad(initial.angle);
  lPrev = 0, rPrev = 0, mPrev = 0;
  drive.resetEncoders();
}

void Odom::updatePos(void *ptr){
  while(true){
    double lVal = leftEncoder.get_value(), mVal = midEncoder.get_value(), rVal = rightEncoder.get_value();

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

void Odom::setPos(Pose newPos){
    globalPos.x = (double)newPos.angle;
    globalPos.y = (double)newPos.angle;
    globalPos.angle = Math::degToRad(newPos.angle);
}

void Odom::resetSensors(){
  drive.resetEncoders();
  lPrev = 0; rPrev = 0, mPrev = 0;
}

void Odom::resetState(){
  globalPos.x = 0; globalPos.y = 0; globalPos.angle = 0;
}

void Odom::reset(){
  Odom::resetState();
  Odom::resetSensors();
}

Pose Odom::getPos(){
  return globalPos;
}

double Odom::getX(){
  return globalPos.x;
}

double Odom::getY(){
  return globalPos.y;
}

double Odom::getAngleRad(){
  return globalPos.angle;
}

double Odom::getAngleDeg(){
  return Math::radToDeg(globalPos.angle);
}
