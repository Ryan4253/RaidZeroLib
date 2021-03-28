#include "main.h"

class LinearMotionProfileController{
  private:
    double accel, decel, maxVel, startVel, endVel;

    LinearMotionProfileController();
    LinearMotionProfileController(double a, double d, double maxVel);
  public:
    LinearMotionProfileController& withStartingVelocity(double v);
    LinearMotionProfileController& withEndingVelocity(double v);
    LinearMotionProfileController& withMaxVelocity(double v);
    LinearMotionProfileController& withMaxAcceleration(double a);
    LinearMotionProfileController& withMaxDeceleration(double a);
    void generateProfile(double d);

};

class TrapezoidalProfileController:LinearMotionProfileController{

};

class SCurveMotionProfileController:LinearMotionProfileController{

};
