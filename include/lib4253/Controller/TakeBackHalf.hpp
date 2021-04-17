#pragma once

class TakeBackHalf{
  private:
    double error, prevError;
    double targetVel, approxVel;
    double output, tbhVal;
    double gain;
    bool firstCross;

  public:
    TakeBackHalf(double g);
    void setGain(double g);
    void setTargetVel(double target);
    void setApproxVel(double approx);
    void initialize();
    double step(double rpm);
};
