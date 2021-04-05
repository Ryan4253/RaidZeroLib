#include "main.h"

PurePursuitFollower::PurePursuitFollower(){

}

void PurePursuitFollower::generateVelocity(){
    std::vector<double> curvature = path.getCurvature();
    std::vector<Vector> waypoint = path.getWaypoint();
    for(int i = 0; i < curvature.size(); i++){
        velocity.push_back(fmin(maxVel, kT / curvature[i]));
    }

    velocity[velocity.size()-1] = 0;

    for(int i = velocity.size()-2; i >= 0; i++){
        double dist = waypoint[i].distanceTo(waypoint[i+1]);
        velocity[i] = fmin(velocity[i], sqrt(velocity[i+1]*velocity[i+1] + 2 * maxAccel * dist));
    }

}

void PurePursuitFollower::initialize(){

}

void PurePursuitFollower::closestPoint(){
    std::vector<Vector> waypoint = path.getWaypoint();
    Vector currentPos = (OdomController::getPos()).toVector();

    double minDist = currentPos.distanceTo(waypoint[prevClosestPt]);
    double minIndex = prevClosestPt;

    for(int i = prevClosestPt; i < waypoint.size(); i++){
        double dist = currentPos.distanceTo(waypoint[i]);
        if(dist < minDist){
          minDist = dist;
          minIndex = i;
        }
    }

    closestPt = minIndex;
}

Vector PurePursuitFollower::lookAhead(){
  Vector start = path.getWaypoint(closestPt);
  Vector end = path.getWaypoint(closestPt+1);
}

void PurePursuitFollower::followPath(Path p){
    path = p;
    generateVelocity();


}

PurePursuitFollower& PurePursuitFollower::withTurnGain(double k){
    kT = k;
    return *this;
}

PurePursuitFollower& PurePursuitFollower::withMaxVel(double v){
    kV = v;
    return *this;
}

PurePursuitFollower& PurePursuitFollower::withMaxAccel(double a){
    kA = a;
    return *this;
}

PurePursuitFollower& PurePursuitFollower::withGain(double v, double a, double p){
    kV = v; kA = a; kP = p;
    return *this;
}
