#pragma once
#include "main.h"


class LinearMotionProfileController{
  private:
    double maxAcc, decel, maxVel;
    double tAcc, tCruise;
    double dAcc, dCruise;
    double dist;

    //double startVel, endVel

    LinearMotionProfileController();
    LinearMotionProfileController(double maxA, double maxVel);
  public:
    /*
    LinearMotionProfileController& withStartingVelocity(double v);
    LinearMotionProfileController& withEndingVelocity(double v);
    LinearMotionProfileController& withMaxVelocity(double v);
    LinearMotionProfileController& withMaxAcceleration(double a);
    LinearMotionProfileController& withMaxDeceleration(double a);
    */

    void setKinematics(double maxV, double maxA);
    void setDistance(double d);
    double getVelocityTime(double t);
    double getVelocityDist(double d);

    double getTotalTime();

};

class TrapezoidalProfileController:LinearMotionProfileController{

};

class SCurveMotionProfileController:LinearMotionProfileController{

};
