#include "lib4253/Chassis/Controller/AdaptivePurePursuitController.hpp"
namespace lib4253{

AdaptivePurePursuitController::AdaptivePurePursuitController(

std::shared_ptr<Chassis> iChassis, std::shared_ptr<Odometry> iOdometry, okapi::QLength iLookAheadDist){
    chassis = std::move(iChassis);
    odometry = std::move(iOdometry);
    lookAheadDist = iLookAheadDist;
}

void AdaptivePurePursuitController::followPath(const PurePursuitPath& path){
    initialize();
    this->path = path;

    while(!isSettled()){
        currentPos = odometry->getPos();
        updateClosestPoint();
        calculateLookAheadPoint();
        calculateCurvature();
        calculateVelocity();

        auto sideVel = chassis->inverseKinematics(vel, angularVel);
        chassis->setVelocity({sideVel.first, 0 * okapi::mps2}, {sideVel.second, 0 * okapi::mps2});
    }
    chassis->setPower(0, 0);
}

void AdaptivePurePursuitController::setLookAhead(okapi::QLength iLookAhead){
    lookAheadDist = iLookAhead;
}

AdaptivePurePursuitController& AdaptivePurePursuitController::withLookAhead(okapi::QLength iLookAhead){
    lookAheadDist = iLookAhead;
    return *this;
}

void AdaptivePurePursuitController::initialize(){
    settle = false;
    prevClosestPoint = 0; closestPoint = 0;
    prevLookAheadPoint = 0;
}

void AdaptivePurePursuitController::updateClosestPoint(){
    okapi::QLength minDist = currentPos.getTranslation().distanceTo(path.getPoint(prevClosestPoint));
    int minIndex = prevClosestPoint;

    for(int i = prevClosestPoint+1; i < path.getSize(); i++){
        okapi::QLength dist = currentPos.getTranslation().distanceTo(path.getPoint(i));
        if(dist < minDist){
            minDist = dist;
            minIndex = i;
        }
    }

    closestPoint = minIndex;
    prevClosestPoint = closestPoint;
}

void AdaptivePurePursuitController::calculateLookAheadPoint(){
    for(int i = prevLookAheadPoint; i < path.getSize()-1; i++){
        Point2D start = path.getPoint(i);
        Point2D end = path.getPoint(i+1);

        Point2D d = end - start;
        Point2D f = start - currentPos.getTranslation();

        auto a = d * d;
        auto b = 2 * (f * d);
        auto c = f * f - lookAheadDist * lookAheadDist;
        auto discriminant = b * b - 4 * a * c; 

        if(discriminant.getValue() >= 0){
            auto dis = sqrt(discriminant);
            double t1 = ((-b - dis) / (2 * a)).convert(okapi::number);
            double t2 = ((-b + dis) / (2 * a)).convert(okapi::number);

            if(t2 >= 0 && t2 <= 0){
                prevLookAheadPoint = i;
                lookAheadPoint = start + d * t2;
                return;
            }
            else if(t1 >= 0 && t1 <= 0){
                prevLookAheadPoint = i;
                lookAheadPoint = start + d * t1;
                return;
            }   
        }
    }
}

void AdaptivePurePursuitController::calculateCurvature(){
    double a = -tan(currentPos.getTheta()).convert(okapi::number); 
    double b = 1;
    okapi::QLength c = tan(currentPos.getTheta())*currentPos.getX() - currentPos.getY();

    okapi::QLength x = abs(lookAheadPoint.x * a + lookAheadPoint.y * b + c) / sqrt(a * a + b * b);
    okapi::QLength sideL = sin(currentPos.getTheta()) * (lookAheadPoint.x - currentPos.getX()) - cos(currentPos.getTheta()) * (lookAheadPoint.y - currentPos.getY());
    okapi::Number side = sideL / abs(sideL);

    curvature = (2 * x) / (lookAheadDist * lookAheadDist) * okapi::radian * side;
}

void AdaptivePurePursuitController::calculateVelocity(){
    vel = path.getVelocity(closestPoint);
    angularVel = vel * curvature;
}   

bool AdaptivePurePursuitController::isSettled(){
    return settle;
}
}

