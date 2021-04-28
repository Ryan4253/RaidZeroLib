#pragma once
#include "main.h"

namespace lib4253{

class PID {
  private:
    double kP, kI, kD;
    double error, prevError, integral, derivative;
    double maxIntegral, minDist;
    double time, prevTime;
    EmaFilter dEMA;

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

}
