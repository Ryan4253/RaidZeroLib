#include "RamseteController.hpp"
#include "lib4253/Utility/Units.hpp"
namespace lib4253{

void RamseteController::followPath(const Trajectory& path){
    for(int i = 0; i < path.getSize(); i++){
        TrajectoryPoint p = path.getKinematics(i);
        std::pair<okapi::QSpeed, okapi::QAngularSpeed> velocity = update(p, odometry->getPos());
        pros::delay(10);
    }
}

std::pair<okapi::QSpeed, okapi::QAngularSpeed> RamseteController::update(const TrajectoryPoint& target, const Pose2D& currentPos){
    Pose2D error = target.position.relativeTo(currentPos);

    // Aliases for equation readability
    double eX = error.getX().convert(okapi::meter);
    double eY = error.getY().convert(okapi::meter);
    double eTheta = error.getTheta().convert(okapi::radian);
    double vRef = target.linVelocity.convert(okapi::mps);
    double omegaRef = target.angVelocity.convert(okapi::radps);

    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * vRef * vRef);

    okapi::QSpeed v{(vRef * cos(eTheta) + k * eX) * okapi::mps};
    okapi::QAngularSpeed omega{(omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY) * okapi::radps};
    return {v, omega};
}

}