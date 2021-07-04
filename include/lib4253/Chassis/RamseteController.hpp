#pragma once
#include "Drive.hpp"
#include "Odometry.hpp"
#include "lib4253/Splines/Trajectory.hpp"
namespace lib4253{

class RamseteController{
    public:
    RamseteController(
        std::shared_ptr<Chassis> iChassis,
        std::shared_ptr<Odometry> iOdom,
        const double& iB,
        const double& iZeta
    );

    void followPath(const Trajectory& path);

    private:
    std::pair<double, double> update;
    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<Odometry> odometry;
    Pose2D tolerance;
    double b;
    double zeta;

    std::pair<okapi::QSpeed, okapi::QAngularSpeed> RamseteController::update(const TrajectoryPoint& target, const Pose2D& currentPos);

};
}