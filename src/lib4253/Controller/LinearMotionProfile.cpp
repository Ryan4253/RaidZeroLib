#include "lib4253/Controller/LinearMotionProfile.hpp"
namespace lib4253{

template<typename Unit>
LinearMotionProfile<Unit>::LinearMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk){
    maxVelocity = iMaxVelocity;
    maxAcceleration = iMaxAcceleration;
    maxJerk = iMaxJerk;
}

template<typename Unit>
bool LinearMotionProfile<Unit>::isInitialized() const{
    return initialize;
}

template<typename Unit>
okapi::QTime LinearMotionProfile<Unit>::getTime() const{
    return totalTime;
}

template<typename Unit>
bool LinearMotionProfile<Unit>::isSettled() const{
    return settled;
}


template<typename Unit>
TrapezoidalMotionProfile<Unit>::TrapezoidalMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk):
LinearMotionProfile<Unit>(iMaxVelocity, iMaxAcceleration, iMaxJerk){
    timePhase.resize(3);
    distPhase.resize(3);
}

template<typename Unit>
void TrapezoidalMotionProfile<Unit>::setDistance(const Distance& iTarget){
    this->targetDist = iTarget;

    timePhase[0] = this->maxVelocity / this->maxAcceleration;
    timePhase[1] = timePhase[0] + (this->targetDist - this->maxAcceleration * timePhase[0] * timePhase[0]) / this->maxVelocity;
    timePhase[2] = timePhase[1] + timePhase[0];
    distPhase[0] = timePhase[0] * this->maxVelocity / 2;
    distPhase[1] = distPhase[0] + this->maxVelocity * (timePhase[1]-timePhase[0]);
    distPhase[2] = this->targetDist;

    this->initialize = true;
}

template<typename Unit> // this is a template function
std::pair<typename TrapezoidalMotionProfile<Unit>::Velocity, typename TrapezoidalMotionProfile<Unit>::Acceleration>
TrapezoidalMotionProfile<Unit>::calculate(const okapi::QTime& currentTime){
    if(currentTime < timePhase[0]){
        return std::make_pair(this->maxAcceleration * currentTime, this->maxAcceleration);
    }
    else if(currentTime > timePhase[1]){
        okapi::QTime tDec = (currentTime - timePhase[1]);
        return std::make_pair(this->maxVelocity - this->maxAcceleration * tDec, -1 * this->maxAcceleration);
    }
    else{
        return std::make_pair(this->maxVelocity, Unit{0.0} / okapi::second / okapi::second);
    }
}

template<typename Unit>
std::pair<typename TrapezoidalMotionProfile<Unit>::Velocity, typename TrapezoidalMotionProfile<Unit>::Acceleration>
TrapezoidalMotionProfile<Unit>::calculate(const Distance& currentDist){
    if(currentDist < distPhase[0]){
        return std::make_pair(this->maxAcceleration * (sqrt(2 * currentDist / this->maxAcceleration)), this->maxAcceleration);
    }
    else if(currentDist > distPhase[1]){
        Distance dDec = distPhase[2] - currentDist;
        return std::make_pair(this->maxAcceleration * (sqrt(2 * currentDist / this->maxAcceleration)), -1 * this->maxAcceleration);
    }
    else{
        return std::make_pair(this->maxVelocity, Unit{0.0} / okapi::second / okapi::second);
    }
}

template<typename Unit>
SCurveMotionProfile<Unit>::SCurveMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk):
LinearMotionProfile<Unit>(iMaxVelocity, iMaxAcceleration, iMaxJerk){
    timePhase.resize(7);
    distPhase.resize(7);
    velPhase.resize(7);
}

template<typename Unit>
void SCurveMotionProfile<Unit>::setDistance(const Distance& iTarget){
    this->targetDist = iTarget;
    
    okapi::QTime jerkTime = this->maxAcceleration / this->maxJerk;
    okapi::QTime accelTime = (this->maxVelocity - jerkTime * this->maxAcceleration) / this->maxAcceleration;

    timePhase[0] = jerkTime;
    timePhase[1] = accelTime;
    timePhase[2] = timePhase[0];

    velPhase[0] = 0.5 * this->maxJerk * jerkTime * jerkTime;
    velPhase[1] = velPhase[0] + this->maxAcceleration * timePhase[1];
    velPhase[2] = this->maxVelocity; // velPhase[1] + this->maxAcceleration * timePhase[2] + -0.5 * this->maxJerk * timePhase[2] * timePhase[2];
    velPhase[3] = velPhase[2];
    velPhase[4] = velPhase[1];
    velPhase[5] = velPhase[0];
    velPhase[6] = Velocity{0.0};

    distPhase[0] = this->maxJerk * timePhase[0] * timePhase[0] * timePhase[0] / 6;
    distPhase[1] = velPhase[0] * timePhase[1] + 0.5 * this->maxAcceleration * timePhase[1] * timePhase[1];
    distPhase[2] = velPhase[1] * timePhase[2] + 0.5 * this->maxAcceleration * timePhase[2] * timePhase[2] + -1 * this->maxJerk * timePhase[2] * timePhase[2] * timePhase[2] / 6;
    distPhase[3] = this->targetDist - 2 * (distPhase[0] + distPhase[1] + distPhase[2]);
    distPhase[4] = distPhase[2];
    distPhase[5] = distPhase[1];
    distPhase[6] = distPhase[0];

    timePhase[3] = distPhase[3] / this->maxVelocity;
    timePhase[4] = timePhase[2];
    timePhase[5] = timePhase[1];
    timePhase[6] = timePhase[0];

    if(distPhase[3] < Unit{0.0}){
        throw std::runtime_error("S-Curve Motion Profile - unable to create profile, distance is too short. Use Trapezoidal instead");
    }

    for(int i = 1; i < 7; i++){
        timePhase[i] = timePhase[i] + timePhase[i-1];
        distPhase[i] = distPhase[i] + distPhase[i-1];
    }

    this->initialize = true;
}

template<typename Unit>
std::pair<typename SCurveMotionProfile<Unit>::Velocity, typename SCurveMotionProfile<Unit>::Acceleration>
SCurveMotionProfile<Unit>::calculate(const okapi::QTime& currentTime){
    if(!this->initialize){
        throw std::runtime_error("S-Curve Motion Profile - distance not set yet!");
    }

    Velocity vel; Acceleration acc;

    if(currentTime < timePhase[0]){
        vel = this->maxJerk * currentTime * currentTime * 0.5;
        acc = this->maxJerk * currentTime; 
    }
    else if(currentTime < timePhase[1]){
        okapi::QTime dTime = currentTime - timePhase[0];
        vel = velPhase[0] + this->maxAcceleration * dTime;
        acc = this->maxAcceleration;
    }
    else if(currentTime < timePhase[2]){
        okapi::QTime dTime = currentTime - timePhase[1];
        vel = velPhase[1] + this->maxAcceleration * dTime + -0.5 * this->maxJerk * dTime * dTime;
        acc = this->maxAcceleration + -1 * this->maxJerk * dTime;
    }
    else if(currentTime < timePhase[3]){
        vel = this->maxVelocity;
        acc = Acceleration{0.0};
    }
    else if(currentTime < timePhase[4]){
        okapi::QTime dTime = currentTime - timePhase[3];
        vel = velPhase[3] + -0.5 * this->maxJerk * dTime * dTime;
        acc = -1 * this->maxJerk * dTime;
    }
    else if(currentTime < timePhase[5]){
        okapi::QTime dTime = currentTime - timePhase[4];
        vel = velPhase[4] + -1 * this->maxAcceleration * dTime;
        acc = -1 * this->maxAcceleration;
    }
    else if(currentTime < timePhase[6]){
        okapi::QTime dTime = currentTime - timePhase[5];
        vel = velPhase[5] + -1 * this->maxAcceleration * dTime + 0.5 * this->maxJerk * dTime * dTime;
        acc = -1 * this->maxAcceleration + this->maxJerk * dTime;
    }
    else{
        vel = Velocity{0.0};
        acc = Acceleration{0.0};
        this->settled = true;
    }

    return std::make_pair(vel, acc);
}

template<typename Unit>
std::pair<typename SCurveMotionProfile<Unit>::Velocity, typename SCurveMotionProfile<Unit>::Acceleration>
SCurveMotionProfile<Unit>::calculate(const Distance& currentDist){
    /*
    if(!this->initialize){
        throw std::runtime_error("S-Curve Motion Profile - distance not set yet!");
    }

    okapi::QTime time;

    if(currentDist < distPhase[0]){
        auto timeCubed = currentDist * 6 / this->maxJerk;
        okapi::RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<3>, std::ratio<0> > times = static_cast<okapi::RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<3>, std::ratio<0>>>(timeCubed);
        time = okapi::root<3>(times);
    }
    else if(currentDist < distPhase[1]){
        Distance dDist = currentDist - distPhase[0];
        
    }
    else if(currentDist < distPhase[2]){

    }
    else if(currentDist < distPhase[3]){
        //vel = this->maxVelocity;
        //acc = Acceleration{0.0};
    }
    else if(currentDist < distPhase[4]){
        //okapi::QTime dTime = currentTime - timePhase[3];
        //vel = velPhase[3] + -0.5 * this->maxJerk * dTime * dTime;
        //acc = -1 * this->maxJerk * dTime;
    }
    else if(currentDist < distPhase[5]){
        //okapi::QTime dTime = currentTime - timePhase[4];
        //vel = velPhase[4] + -1 * this->maxAcceleration * dTime;
        //acc = -1 * this->maxAcceleration;
    }
    else if(currentDist < distPhase[6]){
        //okapi::QTime dTime = currentTime - timePhase[5];
        //vel = velPhase[5] + -1 * this->maxAcceleration * dTime + 0.5 * this->maxJerk * dTime * dTime;
        //acc = -1 * this->maxAcceleration + this->maxJerk * dTime;
    }
    else{
        //vel = Velocity{0.0};
        //acc = Acceleration{0.0};
        //this->settled = true;
    }

    return calculate(time);
    */
   return std::make_pair(Velocity{0.0}, Acceleration{0.0});
}



template class LinearMotionProfile<okapi::QLength>;
template class LinearMotionProfile<okapi::QAngle>;
template class TrapezoidalMotionProfile<okapi::QLength>;
template class TrapezoidalMotionProfile<okapi::QAngle>;
template class SCurveMotionProfile<okapi::QLength>;
template class SCurveMotionProfile<okapi::QAngle>;


}



