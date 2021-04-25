#include "main.h"

class MotorVelocityController{
    double kV, kA, kP;
    public:
      void setGain(double a, double b, double c);
      double calcPower(TrajectoryPoint v, double currentRPM);
};
