#include "lib4253/Splines/Trajectory.hpp"
namespace lib4253{

TrajectoryPoint::TrajectoryPoint(const Pose2D& p, 
                                 const okapi::QSpeed& linV, 
                                 const okapi::QAcceleration& linA, 
                                 const okapi::QAngularSpeed& angV, 
                                 const okapi::QAngularAcceleration& angA):
position(p), linVelocity(linV), linAcceleration(linA), angVelocity(angV), angAcceleration(angA)
{}

Trajectory::Trajectory(const std::vector<TrajectoryPoint>& path):traj(path)
{}

int Trajectory::getSize() const{
    return traj.size();
}

TrajectoryPoint Trajectory::getKinematics(int index) const{
    if(index >= traj.size()){
        TrajectoryPoint ret(Pose2D(0 * okapi::meter, 0 * okapi::meter, {0 * okapi::radian}), 
                            0 * okapi::mps,
                            0 * okapi::mps2,
                            0 * okapi::radps,
                            0 * okapi::radps2);

    }
    else{
        return traj[index];
    }
}
}