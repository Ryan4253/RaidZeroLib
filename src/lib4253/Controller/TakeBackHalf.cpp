#include "lib4253/Controller/TakeBackHalf.hpp"
#include <algorithm>
#include <cmath>
namespace lib4253{

TakeBackHalf::TakeBackHalf(const TBHGain& gain){
    this->gain = gain;
}

void TakeBackHalf::reset(){
    firstCross = true;
    output = 0, tbh = 0;
    error = 0, prevError = 0;
}

void TakeBackHalf::initialize(){
    reset();
}

double TakeBackHalf::step(const double& val){
    error = val;
    output += error * gain.gain;

    if(signbit(error) != signbit(prevError)){
        if(firstCross){
            output = gain.approxVel;
            firstCross = false;
        }
        else{
            output = 0.5 * (output + tbh);
        }
        tbh = output;
    }
    prevError = error;
    return output;
}   
}

IterativeVelTBHController::IterativeVelTBHController(double iGain, 
                                                     double iApproxVel, 
                                                     std::unique_ptr<okapi::VelMath> iVelMath,
                                                     const okapi::TimeUtil& iTimeUtil,
                                                     std::unique_ptr<okapi::Filter> iDerivativeFilter,
                                                     std::shared_ptr<okapi::Logger> iLogger)
    : IterativeVelTBHController({iGain, iApproxVel}, 
                                std::move(iVelMath),
                                iTimeUtil,
                                std::move(iDerivativeFilter),
                                std::move(iLogger)) {

}

IterativeVelTBHController::IterativeVelTBHController(const Gains& iGains,
                                                     std::unique_ptr<okapi::VelMath> iVelMath,
                                                     const okapi::TimeUtil& iTimeUtil,
                                                     std::unique_ptr<okapi::Filter> iDerivativeFilter,
                                                     std::shared_ptr<okapi::Logger> iLogger)
    : logger(std::move(iLogger)),
      velMath(std::move(iVelMath)),
      derivativeFilter(std::move(iDerivativeFilter)),
      loopDtTimer(std::move(iTimeUtil.getTimer())),
      settledUtil(std::move(iTimeUtil.getSettledUtil())){
    setOutputLimits(1, -1);
    setGains(iGains);
}

void IterativeVelTBHController::setSampleTime(okapi::QTime iSampleTime){
    if(iSampleTime > 0 * okapi::millisecond){
        sampleTime = iSampleTime;
    }
}

void IterativeVelTBHController::setOutputLimits(double imax, double imin){
    if(imin > imax){
        const double temp = imax;
        imax = imin;
        imin = temp;
    }

    outputMax = imax;
    outputMin = imin;

    outputSum = std::max()
};



double IterativeVelTBHController::step(double inewReading){

}

void IterativeVelTBHController::setTarget(double itarget){

}

void IterativeVelTBHController::controllerSet(double ivalue) override{

}

double IterativeVelTBHController::getTarget() override;

double IterativeVelTBHController::getTarget() const;

double IterativeVelTBHController::getProcessValue() const override;

double IterativeVelTBHController::getOutput() const override;

double IterativeVelTBHController::getMaxOutput() override;

double IterativeVelTBHController::getMinOutput() override;

double IterativeVelTBHController::getError() const override;

bool IterativeVelTBHController::isSettled() override;




void IterativeVelTBHController::setControllerSetTargetLimits(double itargetMax, double itargetMin) override;

void IterativeVelTBHController::reset() override;

void IterativeVelTBHController::flipDisable() override;

void IterativeVelTBHController::flipDisable(bool iisDisabled) override;

bool IterativeVelTBHController::isDisabled() const override;

okapi::QTime IterativeVelTBHController::getSampleTime() const override;

virtual okapi::QAngularSpeed IterativeVelTBHController::stepVel(double inewReading);

virtual void IterativeVelTBHController::setGains(const Gains &igains);

Gains IterativeVelTBHController::getGains() const;

virtual void IterativeVelTBHController::setTicksPerRev(double tpr);

virtual okapi::QAngularSpeed IterativeVelTBHController::getVel() const;