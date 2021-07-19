#include "RamseteController.hpp"
namespace lib4253{

RamseteController::RamseteController(std::shared_ptr<Chassis> iChassis,
        std::shared_ptr<Odometry> iOdometry,
        const double& iB,
        const double& iZeta
    ){
        b = iB;
        zeta = iZeta;
        chassis = iChassis;
        odometry = iOdometry;
    }

void RamseteController::followPath(const Trajectory& path){
    for(int i = 0; i < path.size(); i++){
        TrajectoryPoint p = path[i];
        std::pair<okapi::QSpeed, okapi::QAngularSpeed> velocity = update(p, odometry->getPos());
        std::pair<okapi::QSpeed, okapi::QSpeed> vel = chassis->inverseKinematics(velocity.first, velocity.second);
        chassis->setVelocity({vel.first, 0 * okapi::mps2}, {vel.second, 0 * okapi::mps2});
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

    // Unit inconsistent dark magic ?!?!??!?!!!??!?!
    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * vRef * vRef);

    okapi::QSpeed v{(vRef * cos(eTheta) + k * eX) * okapi::mps};
    okapi::QAngularSpeed omega{(omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY) * okapi::radps};
    return {v, omega};
}

}