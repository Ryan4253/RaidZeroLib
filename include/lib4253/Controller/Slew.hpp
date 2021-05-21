#pragma once
#include "main.h"

class SlewController{
  protected:
    int speed;
    int accStep;
    int decStep;

  public:
    SlewController(int accel, int decel);
    SlewController();
    void setStep(double a, double d);
    void reset();
    double step(double target);
};
