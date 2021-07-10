#pragma once
#include "lib4253/Chassis/Device/Chassis.hpp"
#include "lib4253/Trajectory/Trajectory.hpp"
#include <map>

namespace lib4253{

class TrajectoryFollower{
    public:
    TrajectoryFollower(std::shared_ptr<Chassis> iChassis);

    ~TrajectoryFollower() = default;

    void addPath(const Trajectory& traj, const std::string& name);

    void deletePath(const std::string& name);

    void followPath(const std::string& name);

    private:
    std::map<std::string, Trajectory> path;
    std::shared_ptr<Chassis> chassis;
};
}