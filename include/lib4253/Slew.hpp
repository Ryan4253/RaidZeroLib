#include "main.h"

class SlewController{
  protected:
    int speed;
    int accStep;
    int decStep;

  public:
    SlewController(int accel, int decel);
    SlewController();
    void reset();
    int step(double target);
};
