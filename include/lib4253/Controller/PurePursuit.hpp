#pragma once
#include "main.h"
#include "lib4253/Splines/SimplePath.hpp"

class PurePursuitFollower{
  private:
    std::vector<double> velocity;
    double maxAcceleration = 0, maxVelocity = 0;
    double kT, kV, kA, kP;
    int prevClosestPt = 0, closestPt = 0;
    SimplePath path;

    void closestPoint(Point2D currentPoint);
    Point2D lookAhead();
    void generateVelocity();

  public:
    PurePursuitFollower();
    void followPath(SimplePath path);
    void initialize();

    void setTurnGain(double k);
    void setKinematics(double v, double a);
    void setGain(double v, double a, double p);

};
