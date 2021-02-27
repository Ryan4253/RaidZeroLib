#include "main.h"

class SlewController{
  protected:
    int speed;
    int accStep;
    int decStep;

  public:
    SlewController(int accel, int decel);
    void reset();
    int step(double target);
};
