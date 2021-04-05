#pragma once
#include "main.h"
#include "lib4253/Filter/emaFilter.hpp"

class PID {
  private:
    double kP, kI, kD;
    double error, prevError, integral, derivative;
    double maxIntegral, minDist;
    double time, prevTime;
    emaFilter dEMA;

  public:
    PID();
    PID(double a, double b, double c);
    PID& withGain(double a, double b, double c);
    PID& withIGain(double windup, double dist);
    PID& withEMAGain(double alpha);

    void initialize();
    double update(double error);
    bool settleUtil(double errorThresh, int timeThresh);
};

class FPID:PID{
  private:
    double kF, target;

  public:
    FPID& withFGain(double f);
    void setTarget(double t);
    double fUpdate(double error);
};
