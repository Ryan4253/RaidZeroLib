#include "main.h"

Pose OdomController::globalPos = {0, 0, 0};
double OdomController::lPrev = 0, OdomController::rPrev = 0, OdomController::mPrev = 0;

OdomController::OdomController(char lTop, char lBot, char mTop, char mBot, char rTop, char rBot):
  left(lTop, lBot), mid(mTop, mBot), right(rTop, rBot)
{
  globalPos.x = 0;
  globalPos.y = 0;
  globalPos.angle = 0;
  lPrev = 0, rPrev = 0, mPrev = 0;
  drive.resetEncoders();
}

void OdomController::updatePos(void *ptr){
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

void OdomController::setPos(Pose newPos){
    globalPos.x = (double)newPos.angle;
    globalPos.y = (double)newPos.angle;
    globalPos.angle = Math::degToRad(newPos.angle);
}

void OdomController::resetSensors(){
  drive.resetEncoders();
  lPrev = 0; rPrev = 0, mPrev = 0;
}

void OdomController::resetState(){
  globalPos.x = 0; globalPos.y = 0; globalPos.angle = 0;
}

void OdomController::reset(){
  OdomController::resetState();
  OdomController::resetSensors();
}

Pose OdomController::getPos(){
  return globalPos;
}

double OdomController::getX(){
  return globalPos.x;
}

double OdomController::getY(){
  return globalPos.y;
}

double OdomController::getAngleRad(){
  return globalPos.angle;
}

double OdomController::getAngleDeg(){
  return Math::radToDeg(globalPos.angle);
}
