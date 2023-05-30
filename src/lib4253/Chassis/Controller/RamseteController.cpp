#include "RamseteController.hpp"
namespace lib4253{

RamseteController::RamseteController(std::shared_ptr<OdomChassisController> iChassis, double iB, double iZeta){
        b = iB;
        zeta = iZeta;
        chassis = std::move(iChassis);
    }

void RamseteController::followPath(const Trajectory& path){
    for(int i = 0; i < path.size(); i++){
        TrajectoryPoint p = path[i];
        Pose currentPos = chassis->getState();
        QSpeed vel = (p.leftVel + p.rightVel) / 2;
        QAngularSpeed angularVel = (p.leftVel-p.rightVel) / chassis->getChassisScales().wheelTrack * radian;
        Pose error = p.position.relativeTo(currentPos);

        auto [leftVel, rightVel] = getTargetVelocity(vel, angularVel, error);

        double leftRPM = Math::velToRPM(leftVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
        double rightRPM = Math::velToRPM(rightVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);


    }
}

std::pair<QSpeed, QSpeed> RamseteController::getTargetVelocity(QSpeed vel, QAngularSpeed angularVel, const Pose& error){
    // Aliases for equation readability
    double eX = error.X().convert(okapi::meter);
    double eY = error.Y().convert(okapi::meter);
    double eTheta = error.Theta().convert(okapi::radian);
    double vRef = vel.convert(okapi::mps);
    double omegaRef = angularVel.convert(okapi::radps);

    // Unit inconsistent dark magic ?!?!??!?!!!??!?!
    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * vRef * vRef);

    QSpeed v{(vRef * cos(eTheta) + k * eX) * okapi::mps};
    QAngularSpeed omega{(omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY) * okapi::radps};

    QSpeed vl = (2 * vl + ((omega.convert(radps) * chassis->getChassisScales().wheelTrack.convert(meter)) * mps)) / 2;
    QSpeed vr = (2 * vl - ((omega.convert(radps) * chassis->getChassisScales().wheelTrack.convert(meter)) * mps)) / 2;

    return {vl, vr};
}


}

