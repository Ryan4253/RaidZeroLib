#pragma once
#include "okapi/api/units/QTime.hpp"
namespace lib4253{

class Settler{
    public:
    Settler();
    Settler(const okapi::QTime& mTime, const double& mError, const double& mDeriv, const okapi::QTime& wTime);
    ~Settler() = default;

    Settler& withMaxTime(const okapi::QTime& time);
    Settler& withMaxError(const double& mError);
    Settler& withMaxDeriv(const double& mDeriv);
    Settler& withWaitTime(const okapi::QTime& time);
    bool isSettled(const uint32_t* currentTime, const double& error);
    
    static Settler makeSettler();
    static Settler getDefaultSettler();

    private:
    double maxTime;
    double maxError;
    double maxDeriv;
    double waitTime;

    double prevError;

    double startTime;
    double exitTime;
};

}