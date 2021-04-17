#pragma once
#include "main.h"

class CustomOdometry{
  protected:
    Pose globalPos;
    virtual void updatePos() = 0;

  public:
    CustomOdometry();
    virtual void withDimensions(std::tuple<double, double> dimension);
    virtual void withDimensions(std::tuple<double, double, double> dimension);

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
    ADIThreeWheelOdometry(std::tuple<char, char, bool> l, std::tuple<char, char, bool> m, std::tuple<char, char, bool> r);
    void withDimensions(std::tuple<double, double, double> dimension);
    void resetSensors();
};

class ADITwoWheelIMUOdometry:public CustomOdometry{
  private:
    void updatePos();
    double mPrev, sPrev, aPrev;
    double mDist, sDist;
    ADIEncoder mid, side; IMU imu;

  public:
    ADITwoWheelIMUOdometry(std::tuple<char, char, bool> s, std::tuple<char, char, bool> m, int port);
    void withDimensions(std::tuple<double, double> dimension);
    void resetSensors();
};
