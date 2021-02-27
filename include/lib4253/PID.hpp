#include "main.h"

class PID {
  protected:
    double kP, kI, kD;
    double error, prevError, integral, derivative, output;
    double deltaT, prevTime;
    Timer time = Timer();
    emaFilter dEMA = emaFilter(1);

  public:
    PID(double a, double b, double c);
    void setGain(double a, double b, double c);
    void initialize();
    double update(double error);
    bool settleUtil(double errorThresh, int timeThresh);
};
