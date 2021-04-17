#pragma once

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
