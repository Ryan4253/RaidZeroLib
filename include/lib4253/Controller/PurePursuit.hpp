#pragma once
#include "main.h"
#include "lib4253/Splines/SimplePath.hpp"

namespace lib4253{

class PurePursuitFollower{
  private:
    SimplePath path;
    std::vector<double> velocity;
    double maxAcceleration = 0, maxVelocity = 0, trackWidth, radius, kT;
    int prevClosestPoint = 0, closestPoint = 0;
    int prevLookAheadPoint = 0; Point2D lookAheadPoint;
    double curvature;
    bool settled = false;

    void generateVelocity();
    void calcClosestPoint(Pose2D currentPos);
    void calcLookAheadPoint(Pose2D currentPos);
    void calcCurvature(Pose2D currentPos);
    std::pair<double, double> calcPower(Pose2D currentPos);

  public:
    void setPath(SimplePath path);
    void initialize();

    std::pair<double, double> step(Pose2D CurrentPos);
    void setTurnGain(double k);
    void setKinematics(double v, double a);
    void setTrackWidth(double size);
    void setLookAhead(double dist);

    bool isSettled();


};

}
