#include "RaidZeroLib/Control/Iterative/IterativeVelBangBangController.hpp"

namespace rz{
using namespace okapi;

IterativeVelBangBangController::IterativeVelBangBangController(Gains iGains, 
                                                     std::unique_ptr<okapi::VelMath> iVelMath,
                                                     const okapi::TimeUtil& iTimeUtil,
                                                     std::shared_ptr<okapi::Logger> iLogger):
    gains(iGains), velMath(std::move(iVelMath)), loopDtTimer(iTimeUtil.getTimer()), settledUtil(iTimeUtil.getSettledUtil()), logger(std::move(iLogger)){
    setOutputLimits(-1, 1);
}

void IterativeVelBangBangController::setSampleTime(okapi::QTime iSampleTime){
    if(iSampleTime > 0 * okapi::millisecond){
        sampleTime = iSampleTime;
    }
}

void IterativeVelBangBangController::setOutputLimits(double iMax, double iMin){
    if(iMin > iMax){
        const double temp = iMax;
        iMax = iMin;
        iMin = temp;
    }

    outputMax = iMax;
    outputMin = iMin;

    output = std::clamp(output, outputMin, outputMax);
};

void IterativeVelBangBangController::setControllerSetTargetLimits(double iTargetMax, double iTargetMin){
    if (iTargetMin > iTargetMax) {
        const double temp = iTargetMax;
        iTargetMax = iTargetMin;
        iTargetMin = temp;
    }

    controllerSetTargetMax = iTargetMax;
    controllerSetTargetMin = iTargetMin;
}

QAngularSpeed IterativeVelBangBangController::stepVel(double iNewReading){
    return velMath->step(iNewReading);
}

double IterativeVelBangBangController::step(double iNewReading){
    if(controllerIsDisabled){
        return 0;
    }

    loopDtTimer->placeHardMark();

    if(loopDtTimer->getDtFromHardMark() >= sampleTime){
        stepVel(iNewReading);
        error = getError();

        if(abs(error) <= gains.deadband){
            output = gains.targetPower;
        }
        else if(error > 0){
            output = gains.highPower;
        }
        else{
            output = gains.lowPower;
        }

        loopDtTimer->clearHardMark();    

        settledUtil->isSettled(error);
    }

    output = std::clamp(output, outputMin, outputMax);
    return output;
}

void IterativeVelBangBangController::setTarget(const double iTarget) {
    LOG_INFO("IterativeVelBangBangController: Set target to " + std::to_string(iTarget));
    target = iTarget;
}

void IterativeVelBangBangController::controllerSet(double iValue){
    target = remapRange(iValue, -1, 1, controllerSetTargetMin, controllerSetTargetMax);
}

double IterativeVelBangBangController::getTarget(){
    return target;
}

double IterativeVelBangBangController::getTarget() const {
    return target;
}

double IterativeVelBangBangController::getProcessValue() const{
    return velMath->getVelocity().convert(rpm);
}

double IterativeVelBangBangController::getOutput() const{
    return isDisabled() ? 0 : output;
}

double IterativeVelBangBangController::getMaxOutput(){
    return outputMax;
}

double IterativeVelBangBangController::getMinOutput(){
    return outputMin;
}

double IterativeVelBangBangController::getError() const{
    return getTarget() - getProcessValue();
}

bool IterativeVelBangBangController::isSettled(){
    return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativeVelBangBangController::reset(){
    LOG_INFO_S("IterativeVelPIDController: Reset");

    error = 0;
    output = 0;
    settledUtil->reset();
}

void IterativeVelBangBangController::flipDisable(){
    flipDisable(!controllerIsDisabled);
}

void IterativeVelBangBangController::flipDisable(const bool iIsDisabled){
    LOG_INFO("IterativeVelPIDController: flipDisable " + std::to_string(iIsDisabled));
    controllerIsDisabled = iIsDisabled;
}

bool IterativeVelBangBangController::isDisabled() const{
    return controllerIsDisabled;
}

void IterativeVelBangBangController::setGains(const Gains& iGains){
    gains = iGains;
}

IterativeVelBangBangController::Gains IterativeVelBangBangController::getGains() const{
    return gains;
}

void IterativeVelBangBangController::setTicksPerRev(double iTPR){
    velMath->setTicksPerRev(iTPR);
}

okapi::QAngularSpeed IterativeVelBangBangController::getVel() const{
    return velMath->getVelocity();
}

okapi::QTime IterativeVelBangBangController::getSampleTime() const{
    return sampleTime;
}


};