#include "lib4253/Splines/Trajectory.hpp"
namespace lib4253{

TrajectoryPoint::TrajectoryPoint(const Point2D& p, const double& linV, const double& linA, const double& angV, const double& angA):
position(p), linVelocity(linV), linAcceleration(linA), angVelocity(angV), angAcceleration(angA)
{}

Trajectory::Trajectory(std::vector<TrajectoryPoint> path):traj(path)
{}

int Trajectory::getSize() const{
    return traj.size();
}

TrajectoryPoint Trajectory::getKinematics(int index){
    if(index >= traj.size()){
        TrajectoryPoint ret(Pose2D(0, 0, 0), 0, 0, 0, 0);

    }
    else{
        return traj[index];
    }
}
}