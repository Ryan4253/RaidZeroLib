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

    void setGain(double a, double b, double c);
    void setIGain(double windup, double dist);
    void setEMAGain(double alpha);

    void initialize();
    double update(double error);
};

class FPID:PID{
  private:
    double kF, target;

  public:
    void setFGain(double f);
    void setTarget(double t);
    double fUpdate(double error);
};
