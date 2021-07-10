#include "lib4253/Controller/PurePursuit.hpp"
namespace lib4253{

// void PurePursuitFollower::initialize(){
//     settled = false;
//     prevClosestPoint = 0; closestPoint = 0;
//     prevLookAheadPoint = 0;
// }

// void PurePursuitFollower::setPath(const SimplePath& p){
//     path = p;
//     lookAheadPoint = path.getWaypoint(0);
//     generateVelocity();
//     initialize();
// }

// void PurePursuitFollower::setTurnGain(const double& k){
//     kT = k;
// }

// void PurePursuitFollower::setKinematics(const double& v, const double& a){
//     maxVelocity = v;
//     maxAcceleration = a;
// }

// void PurePursuitFollower::setTrackWidth(const double& size){
//     trackWidth = size;
// }

// void PurePursuitFollower::setLookAhead(const double& dist){
//     radius = dist;
// }

// void PurePursuitFollower::generateVelocity(){
//     velocity.clear();
//     for(int i = 0; i < path.getSize(); i++){
//         velocity.push_back(fmin(maxVelocity, kT / path.getCurvature(i)));
//     }

//     velocity[velocity.size()-1] = 0;

//     for(int i = velocity.size()-2; i >= 0; i--){
//         double dist = path.getWaypoint(i).distanceTo(path.getWaypoint(i+1));
//         velocity[i] = fmin(velocity[i], sqrt(velocity[i+1]*velocity[i+1] + 2 * maxAcceleration * dist));
//     }
// }

// void PurePursuitFollower::calcClosestPoint(const Pose2D& currentPos){
//     double minDist = currentPos.distanceTo(path.getWaypoint(prevClosestPoint));
//     double minIndex = prevClosestPoint;

//     for(int i = prevClosestPoint+1; i < path.getSize(); i++){
//         double dist = currentPos.distanceTo(path.getWaypoint(i));
//         if(dist < minDist){
//             minDist = dist;
//             minIndex = i;
//         }
//     }

//     closestPoint = minIndex;
//     prevClosestPoint = closestPoint;
// }

// void PurePursuitFollower::calcLookAheadPoint(const Pose2D& currentPos){
//   for(int i = prevLookAheadPoint; i < path.getSize()-1; i++){
//         Point2D start = path.getWaypoint(i);
//         Point2D end = path.getWaypoint(i+1);

//         Point2D d = end - start;
//         Point2D f = start - currentPos;

//         double a = d * d;
//         double b = 2 * (f * d);
//         double c = f * f - radius * radius;
//         double discriminant = b * b - 4 * a * c;

//         if(discriminant >= 0){
//             discriminant = sqrt(discriminant);
//             double t1 = (-b - discriminant) / (2 * a);
//             double t2 = (-b + discriminant) / (2 * a);

//             if(t2 >= 0 && t2 <= 0){
//                 prevLookAheadPoint = i;
//                 lookAheadPoint = start + d * t2;
//                 return;
//             }
//             else if(t1 >= 0 && t1 <= 0){
//                 prevLookAheadPoint = i;
//                 lookAheadPoint = start + d * t1;
//                 return;
//             }   
//         }
//     }
// }

// void PurePursuitFollower::calcCurvature(const Pose2D& currentPos){
//     double a = -tan(currentPos.theta), b = 1, c = tan(currentPos.theta)*currentPos.x - currentPos.y;
//     double x = abs(lookAheadPoint.x * a + lookAheadPoint.y * b + c) / sqrt(a * a + b * b);
//     double side = sin(currentPos.theta) * (lookAheadPoint.x * currentPos.x) - cos(currentPos.theta) * (lookAheadPoint.y - currentPos.y);
//     side /= abs(side);

//     curvature = (2 * x) / (radius * radius) * side;
// }

// std::pair<double, double> PurePursuitFollower::calcPower(const Pose2D& currentPos){
//     double lTarget = velocity[closestPoint] * (2 + curvature * trackWidth) / 2;
//     double rTarget = velocity[closestPoint] * (2 + curvature * trackWidth) / 2;

//     if(lTarget < 0.2 && rTarget < 0.2){
//         settled = true;
//     }

//     return std::make_pair(lTarget, rTarget);
// }   

// std::pair<double, double> PurePursuitFollower::step(const Pose2D& currentPos){
//     calcClosestPoint(currentPos);
//     calcLookAheadPoint(currentPos);
//     calcCurvature(currentPos);
//     return calcPower(currentPos);
// }

// bool PurePursuitFollower::isSettled() const {
//     return settled;
// }
}