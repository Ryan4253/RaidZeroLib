#include "RamseteController.hpp"
#include "lib4253/Utility/Units.hpp"
namespace lib4253{

void RamseteController::followPath(const Trajectory& path){
    for(int i = 0; i < path.getSize(); i++){
        TrajectoryPoint p = path.getKinematics(i);
        std::pair<double, double> velocity = update(p, odometry->getPos());
        pros::delay(10);
    }
}

std::pair<double, double> RamseteController::update(const TrajectoryPoint& target, const Pose2D& currentPos){
    Pose2D error = target.position - currentPos;

    // Aliases for equation readability
    double eX = error.x;
    double eY = error.y;
    double eTheta = error.theta;
    double vRef = target.linVelocity;
    double omegaRef = target.angVelocity;

    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * vRef * vRef);

    double v{vRef * cos(eTheta) + k * eX};
    double omega{omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY};
    return {v, omega};
}

}