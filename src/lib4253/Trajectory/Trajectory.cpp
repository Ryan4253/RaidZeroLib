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

TrajectoryPoint::TrajectoryPoint(double iLeftDist, double iRightDist,
            double iLeftVel, double iRightVel,
            double iLeftAccel, double iRightAccel,
            double iX, double iY, double iTheta){
    leftDist = iLeftDist * foot, rightDist = iRightDist * foot;
    leftVel = iLeftVel * ftps, rightVel = iRightVel * ftps;
    leftAccel = iLeftAccel * ftps2, rightAccel = iRightAccel * ftps2;
    position = Pose(iX * foot, iY * foot, iTheta * degree);
}

std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt){
    os << "(" << pt.position.X().convert(foot) << ", " << pt.position.Y().convert(foot) << ", " << pt.position.Theta().convert(degree) << "),  "
       << pt.leftDist.convert(foot) << " " << pt.rightDist.convert(foot) << " " 
       << pt.leftVel.convert(ftps) << " " << pt.rightVel.convert(ftps) << " "
       << pt.leftAccel.convert(ftps2) << " " << pt.rightAccel.convert(ftps2) << std::endl;

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