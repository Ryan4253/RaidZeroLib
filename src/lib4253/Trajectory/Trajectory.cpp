#include "lib4253/Trajectory/Trajectory.hpp"
namespace lib4253{

TrajectoryPoint::TrajectoryPoint(const Pose2D& p, 
                                 const okapi::QSpeed& linV, 
                                 const okapi::QAcceleration& linA, 
                                 const okapi::QAngularSpeed& angV, 
                                 const okapi::QAngularAcceleration& angA):
position(p), linVelocity(linV), linAcceleration(linA), angVelocity(angV), angAcceleration(angA)
{}

}