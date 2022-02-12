#include "Trajectory.hpp"

namespace lib4253{
using namespace okapi;


TrajectoryPoint::TrajectoryPoint(const QLength iLeftDist, const QLength iRightDist,
                const QSpeed iLeftVel, const QSpeed iRightVel,
                const QAcceleration iLeftAccel, const QAcceleration iRightAccel,
                const Pose& iPosition): 
    leftDist(iLeftDist), rightDist(iRightDist), 
    leftVel(iLeftVel), rightVel(iRightVel), 
    leftAccel(iLeftAccel), rightAccel(iRightAccel), 
    position(iPosition){}

std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt){
    os << pt.position.X().convert(meter) << " " << pt.position.Y().convert(meter) << " " << pt.position.Theta().convert(radian) << " "
       << pt.leftDist.convert(meter) << " " << pt.rightDist.convert(meter) << " " 
       << pt.leftVel.convert(mps) << " " << pt.rightVel.convert(mps) << " "
       << pt.leftAccel.convert(mps2) << " " << pt.rightAccel.convert(mps2) << std::endl;

    return os;
}

Trajectory::Trajectory(const std::initializer_list<TrajectoryPoint>& iPath): path(iPath){}
    
TrajectoryPoint Trajectory::operator[](int index) const{
    return path[index];
}

int Trajectory::size() const{
    return path.size();
}

}