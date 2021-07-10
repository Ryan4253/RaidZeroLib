#pragma once
#include "Drive.hpp"
#include "Odometry.hpp"
#include "lib4253/Splines/Trajectory.hpp"
#include "lib4253/Utility/Units.hpp"
namespace lib4253{

class RamseteController{
    public:
    RamseteController(
        std::shared_ptr<Chassis> iChassis,
        std::shared_ptr<Odometry> iOdometry,
        const double& iB,
        const double& iZeta
    );

    void followPath(const Trajectory& path);

    private:
    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<Odometry> odometry;
    Pose2D tolerance;
    double b;
    double zeta;

    std::pair<okapi::QSpeed, okapi::QAngularSpeed> update(const TrajectoryPoint& target, const Pose2D& currentPos);

};
}