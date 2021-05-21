#pragma once
#include "main.h"

struct TrajectoryPoint{
  double velocity;
  double acceleration;

  TrajectoryPoint(double v, double a);
};

class Trajectory{
  private:
    std::vector<TrajectoryPoint> left;
    std::vector<TrajectoryPoint> right;

  public:
    Trajectory() = default;
    Trajectory(std::vector<TrajectoryPoint> l, std::vector<TrajectoryPoint> r);
    int getSize();
    std::pair<TrajectoryPoint, TrajectoryPoint> getKinematics(int index);
};
