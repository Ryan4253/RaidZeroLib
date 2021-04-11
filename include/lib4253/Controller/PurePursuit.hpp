#pragma once
#include "main.h"
#include "lib4253/Utility/Path.hpp"

class PurePursuitFollower{
  private:
    std::vector<double> velocity;
    double maxAcceleration = 0, maxVelocity = 0;
    double kT, kV, kA, kP;
    int prevClosestPt = 0, closestPt = 0;
    Path path;

    void closestPoint();
    Vector lookAhead();
    void generateVelocity();

  public:
    PurePursuitFollower();
    void followPath(Path path);
    void initialize();

    void setTurnGain(double k);
    void setKinematics(double v, double a);
    void setGain(double v, double a, double p);

};
