#include "Settler.hpp"
namespace lib4253{

Settler::Settler(){
    maxTime = 10000000;
    maxError = 1<<30;
    maxDeriv = 1<<30;
    waitTime = 0;
    startTime = -1;
    exitTime = -1;
    prevError = 1<<30;
}

Settler::Settler(const okapi::QTime& mTime, const double& mError, const double& mDeriv, const okapi::QTime& wTime){
    maxTime = mTime.convert(okapi::millisecond);
    maxError = mError;
    maxDeriv = mDeriv;
    waitTime = wTime.convert(okapi::millisecond);
    startTime = -1;
    exitTime = -1;
    prevError = 1<<30;
}

Settler& Settler::withMaxTime(const okapi::QTime& time){
    maxTime = time.convert(okapi::millisecond);
    return *this;
}

Settler& Settler::withMaxError(const double& mError){
    maxError = mError;
    return *this;
}

Settler& Settler::withMaxDeriv(const double& mDeriv){
    maxDeriv = mDeriv;
    return *this;
}

Settler& Settler::wait(const okapi::QTime& time){
    waitTime = time.convert(okapi::millisecond);
    return *this;
}

bool Settler::isSettled(const uint32_t* currentTime, const double& error){
    if(startTime == -1){
        startTime = *currentTime;
    }

    if((*currentTime - startTime) >= maxTime){
        return true;
    }

    double derivative = error - prevError;
    prevError = error;

    if(std::fabs(error) <= maxError && std::fabs(derivative) <= maxDeriv){
        if(exitTime == -1){
            exitTime = *currentTime;
        }

        return ((*currentTime - exitTime) >= waitTime) ? true : false;
    }
    else{
        exitTime = -1;
        return false;
    }
}

Settler Settler::makeSettler(){
    Settler ret;
    return ret;
}

Settler Settler::getDefaultSettler(){
    Settler ret(100000 * okapi::second, 0.5, 0.1, 200 * okapi::millisecond);
    return ret;
}
}

