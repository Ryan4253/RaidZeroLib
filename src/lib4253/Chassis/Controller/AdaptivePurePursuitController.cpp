#include "lib4253/Chassis/Controller/AdaptivePurePursuitController.hpp"
namespace lib4253{

AsyncAdaptivePurePursuitController::AsyncAdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& iChassis, 
                                                             const PurePursuitGains& iGains,
                                                             QLength iLookAhead,
                                                             const MotorFFController& iLeftController,
                                                             const MotorFFController& iRightController,
                                                             const TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    leftMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();
    rightMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();
    gains = iGains;
    lookAhead = iLookAhead;
    leftController = iLeftController;
    rightController = iRightController;
    
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());
    this->startTask();
}

AsyncAdaptivePurePursuitController::AsyncAdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& iChassis, 
                                                             const PurePursuitGains& iGains,
                                                             QLength iLookAhead,
                                                             const TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    leftMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();
    rightMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();
    gains = iGains;
    lookAhead = iLookAhead;
    leftController = std::nullopt;
    rightController = std::nullopt;

    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());
    this->startTask();
}

void AsyncAdaptivePurePursuitController::followPath(const DiscretePath& iPath){
    lock.take(5);
    initialize();
    path = PurePursuitPath(iPath, gains);
    lock.give();
}

void AsyncAdaptivePurePursuitController::setLookAhead(QLength iLookAhead){
    lock.take(5);
    lookAhead = iLookAhead;
    lock.give();
}

void AsyncAdaptivePurePursuitController::setGains(const PurePursuitGains& iGains){
    lock.take(5);
    gains = iGains;
    lock.give();
}

void AsyncAdaptivePurePursuitController::initialize(){
    settled = false;
    prevClosest = std::nullopt; 
    prevLookAheadIndex = 0;
    prevLookAheadT = 0;
}

std::optional<double> AsyncAdaptivePurePursuitController::getT(const Point& iStart, const Point& iEnd, const Pose& iPos){
    Point d = iEnd - iStart;
    Point f = iStart - iPos.getTranslation();

    auto a = d * d;
    auto b = 2 * (f * d);
    auto c = f * f - lookAhead * lookAhead;
    auto discriminant = b * b - 4 * a * c; 

    if(discriminant.getValue() >= 0){
        auto dis = sqrt(discriminant);
        double t1 = ((-b - dis) / (2 * a)).convert(okapi::number);
        double t2 = ((-b + dis) / (2 * a)).convert(okapi::number);

        if(t2 >= 0 && t2 <= 1){
            return t2;
        }
        else if(t1 >= 0 && t1 <= 1){
            return t1;
        }   
    }

    return std::nullopt;
}

int AsyncAdaptivePurePursuitController::getClosestPoint(const Pose& currentPos){
    QLength minDist {std::numeric_limits<double>::max()};
    int closest = prevClosest.value_or(0);

    for(int i = closest; i < path.size(); i++){
        QLength dist = currentPos.getTranslation().distTo(path.getPoint(i));
        if(dist < minDist){
            minDist = dist;
            closest = i;
        }
    }

    prevClosest = closest;
    return closest;
}

Point AsyncAdaptivePurePursuitController::getLookAheadPoint(const Pose& currentPos){

    int closestIndex = prevClosest.value_or(0);


    for(int i = std::max(closestIndex, prevLookAheadIndex); i < path.size()-1; i++){
        Point start = path.getPoint(i);
        Point end = path.getPoint(i+1);

        auto t = getT(start, end, currentPos);

        if(t){
            if(i > prevLookAheadIndex || t.value() > prevLookAheadT){
                prevLookAheadIndex = i;
                prevLookAheadT = t.value();
                break;
            }
        }
    }

    return path[prevLookAheadIndex] + (path[prevLookAheadIndex+1] - path[prevLookAheadIndex]) * prevLookAheadT;
}

QCurvature AsyncAdaptivePurePursuitController::calcCurvature(const Pose& iPos, const Point& lookAheadPt){
    double a = -tan(iPos.Theta()).convert(okapi::number); 
    double b = 1;
    QLength c = tan(iPos.Theta())*iPos.X() - iPos.Y();

    QLength x = abs(lookAheadPt.X() * a + lookAheadPt.Y() * b + c) / sqrt(a * a + b * b);
    QLength sideL = sin(iPos.Theta()) * (lookAheadPt.X() - iPos.X()) - cos(iPos.Theta()) * (lookAheadPt.Y() - iPos.Y());
    Number side = sideL / abs(sideL);

    if(sideL.convert(meter) == 0){
        return 0 * radpm;
    }

    return (2 * x) / (lookAhead * lookAhead) * okapi::radian * side;
}

std::pair<QSpeed, QSpeed> AsyncAdaptivePurePursuitController::calcVelocity(QCurvature iCurvature, int iClosestPt){
    QSpeed vel = isReversed ? -path.getVelocity(iClosestPt) : path.getVelocity(iClosestPt);
    QSpeed vl = vel * (2+iCurvature.convert(radpm)*chassis->getChassisScales().wheelTrack.convert(meter)) / 2;
    QSpeed vr = vel * (2-iCurvature.convert(radpm)*chassis->getChassisScales().wheelTrack.convert(meter)) / 2;

    if(isReversed){
        return {vr, vl};
    }
    else{
        return {vl, vr};
    }
}   

std::pair<QAcceleration, QAcceleration> AsyncAdaptivePurePursuitController::calcAcceleration(QCurvature iCurvature, int iClosestPt){
    QAcceleration accel = isReversed ? -path.getAcceleration(iClosestPt) : path.getAcceleration(iClosestPt);
    QAcceleration al = accel * (2+iCurvature.convert(radpm)*chassis->getChassisScales().wheelTrack.convert(meter)) / 2;
    QAcceleration ar = accel * (2-iCurvature.convert(radpm)*chassis->getChassisScales().wheelTrack.convert(meter)) / 2;

    if(isReversed){
        return {ar, al};
    }
    else{
        return {al, ar};
    }
}   

bool AsyncAdaptivePurePursuitController::isSettled(){
    return settled;
}

void AsyncAdaptivePurePursuitController::waitUntilSettled(){
    while(!settled){
        pros::delay(10);
    }
}

void AsyncAdaptivePurePursuitController::loop(){
    while(true){
        lock.take(5);
        if(settled){
            lock.give();
            rate->delayUntil(10);
            continue;
        }

        Pose pos = chassis->getState();
        int closest = getClosestPoint(pos);
        Point lookAheadPt = getLookAheadPoint(pos);
        QCurvature curvature = calcCurvature(pos, lookAheadPt);
        std::pair<QSpeed, QSpeed> vel = calcVelocity(curvature, closest);
        std::pair<QAcceleration, QAcceleration> accel = calcAcceleration(curvature, closest);

        if(leftController && rightController){
            double leftPower = leftController->step(0 * meter, vel.first, accel.first, 0 * meter, 0 * mps);
            double rightPower = leftController->step(0 * meter, vel.first, accel.first, 0 * meter, 0 * mps);
            chassis->getModel()->tank(leftPower, rightPower);
        }
        else{
            
        }
        lock.give();
        rate->delayUntil(10);
    }
}
}
