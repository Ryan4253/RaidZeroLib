#pragma once
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "lib4253/Trajectory/Trajectory.hpp"
#include "lib4253/Utility/Units.hpp"
#include "lib4253/Utility/Math.hpp"
#include <memory>
namespace lib4253{
using namespace okapi;

class RamseteController{
    public:
    RamseteController(
        std::shared_ptr<OdomChassisController> iChassis,
        double iB = 2.0,
        double iZeta = 0.7
    );

    void followPath(const Trajectory& path);

    private:
    std::shared_ptr<OdomChassisController> chassis;
    Pose tolerance;
    double b;
    double zeta;

    std::pair<QSpeed, QSpeed> getTargetVelocity(QSpeed vel, QAngularSpeed angularVel, const Pose& error);

};
}