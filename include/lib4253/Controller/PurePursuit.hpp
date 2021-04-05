#pragma once
#include "main.h"
#include "lib4253/Utility/Path.hpp"

class PurePursuitFollower{
  private:
    std::vector<double> velocity;
    double maxAccel = 0, maxVel = 0;
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
    PurePursuitFollower& withTurnGain(double k);
    PurePursuitFollower& withMaxVel(double v);
    PurePursuitFollower& withMaxAccel(double a);
    PurePursuitFollower& withGain(double v, double a, double p);

};
