#pragma once
#include "main.h"

class OdomController{
  private:
    static Pose globalPos;
    static double lPrev, rPrev, mPrev;
    ADIEncoder left, mid, right;

  public:
    OdomController(char lTop, char lBot, char mTop, char mBot, char rTop, char rBot);
    static void updatePos(void *ptr);

    static void resetSensors();
    static void resetState();
    static void reset();

    static void setPos(Pose newPos);
    static void setX(double newx);
    static void setY(double newy);
    static void setAngle(double theta);

    static Pose getPos();
    static double getX();
    static double getY();
    static double getAngleRad();
    static double getAngleDeg();
};
