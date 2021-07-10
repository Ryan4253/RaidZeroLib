#include "TrajectoryFollower.hpp"
namespace lib4253{

TrajectoryFollower::TrajectoryFollower(std::shared_ptr<Chassis> iChassis){
    chassis = iChassis;
}

void TrajectoryFollower::addPath(const Trajectory& traj, const std::string& name){
    if(path.find(name) == path.end()){
        throw std::invalid_argument("The username has already been taken");
    }
    path.insert(make_pair(name, traj));
}

void TrajectoryFollower::deletePath(const std::string& name){
    auto pos = path.find(name);
    if (pos != path.end()){
        path.erase (pos);
    }
}

void TrajectoryFollower::followPath(const std::string& name){
    if(path.find(name) == path.end()){
        throw std::invalid_argument("Path not found, check the name");
    }

    Trajectory trajectory = path[name];

    for(int i = 0; i < trajectory.size(); i++){
        TrajectoryPoint kinematics = trajectory[i];
        std::pair<okapi::QSpeed, okapi::QSpeed> vel = chassis->inverseKinematics(kinematics.linVelocity, kinematics.angVelocity);
        std::pair<okapi::QAcceleration, okapi::QAcceleration> accel = chassis->inverseKinematics(kinematics.linAcceleration, kinematics.angAcceleration);
        chassis->setVelocity({vel.first, accel.first}, {vel.second, accel.second});
        pros::delay(10);
    }

    chassis->setPower(0, 0);
}

}