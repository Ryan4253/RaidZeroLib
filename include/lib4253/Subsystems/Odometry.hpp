#pragma once
#include "main.h"

class CustomOdometry{
  protected:
    Pose globalPos;
    virtual void updatePos() = 0;

  public:
    CustomOdometry();

    Pose getPos();
    double getX();
    QLength getQX();
    double getY();
    QLength getQY();
    double getAngleDeg();
    double getAngleRad();

    void setPos(Pose newPos);
    void setX(double x);
    void setX(QLength );
    void setY(double y);
    void setY(QLength inch);
    void setAngleDeg(double theta);
    void setAngleRad(double theta);

    void resetState();
    virtual void resetSensors() = 0;
    void reset();
    static void odomTask(void *ptr);
};

class ADIThreeWheelOdometry:public CustomOdometry{
  private:
    void updatePos();
    double lPrev, mPrev, rPrev;
    double lDist, rDist, mDist;
    ADIEncoder left, mid, right;

  public:
    ADIThreeWheelOdometry& withDimensions(std::tuple<double, double, double> dimension);
    ADIThreeWheelOdometry(std::tuple<char, char, bool> l, std::tuple<char, char, bool> m, std::tuple<char, char, bool> r);
    void resetSensors();
};

extern CustomOdometry* tracker;
