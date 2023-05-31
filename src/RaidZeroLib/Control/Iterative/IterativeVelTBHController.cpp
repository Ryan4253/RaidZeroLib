#include "RaidZeroLib/Control/Iterative/IterativeVelTBHController.hpp"

namespace rz{

IterativeVelTBHController::IterativeVelTBHController(double iGain, 
                                                     std::unique_ptr<okapi::VelMath> iVelMath,
                                                     const okapi::TimeUtil& iTimeUtil,
                                                     std::shared_ptr<okapi::Logger> iLogger):
    gain(iGain), velMath(std::move(iVelMath)), loopDtTimer(iTimeUtil.getTimer()), settledUtil(iTimeUtil.getSettledUtil()), logger(std::move(iLogger)){
    setOutputLimits(-1, 1);
}

void IterativeVelTBHController::setSampleTime(okapi::QTime iSampleTime){
    if(iSampleTime > 0 * okapi::millisecond){
        sampleTime = iSampleTime;
    }
}

void IterativeVelTBHController::setOutputLimits(double iMax, double iMin){
    if(iMin > iMax){
        const double temp = iMax;
        iMax = iMin;
        iMin = temp;
    }

    outputMax = iMax;
    outputMin = iMin;

    output = std::clamp(output, outputMin, outputMax);
};

void IterativeVelTBHController::setControllerSetTargetLimits(double iTargetMax, double iTargetMin){
    if (iTargetMin > iTargetMax) {
        const double temp = iTargetMax;
        iTargetMax = iTargetMin;
        iTargetMin = temp;
    }

    controllerSetTargetMax = iTargetMax;
    controllerSetTargetMin = iTargetMin;
}

QAngularSpeed IterativeVelTBHController::stepVel(double iNewReading){
    return velMath->step(iNewReading);
}

double IterativeVelTBHController::step(double iNewReading){
    if(controllerIsDisabled){
        return 0;
    }

    loopDtTimer->placeHardMark();

    if(loopDtTimer->getDtFromHardMark() >= sampleTime){
        stepVel(iNewReading);
        error = getError();

        output += error;
        if(signbit(error) != signbit(prevError)){
            output = 0.5 * (output + tbh);
            tbh = output;
        }
        prevError = error;

        loopDtTimer->clearHardMark();    

        settledUtil->isSettled(error);
    }

    output = std::clamp(output, outputMin, outputMax);
    return output;
}

void IterativeVelTBHController::setTarget(const double iTarget) {
    LOG_INFO("IterativeVelTBHController: Set target to " + std::to_string(iTarget));
    target = iTarget;
}

void IterativeVelTBHController::controllerSet(double iValue){
    target = remapRange(iValue, -1, 1, controllerSetTargetMin, controllerSetTargetMax);
}

double IterativeVelTBHController::getTarget(){
    return target;
}

double IterativeVelTBHController::getTarget() const {
    return target;
}

double IterativeVelTBHController::getProcessValue() const{
    return velMath->getVelocity().convert(rpm);
}

double IterativeVelTBHController::getOutput() const{
    return isDisabled() ? 0 : output;
}

double IterativeVelTBHController::getMaxOutput(){
    return outputMax;
}

double IterativeVelTBHController::getMinOutput(){
    return outputMin;
}

double IterativeVelTBHController::getError() const{
    return getTarget() - getProcessValue();
}

bool IterativeVelTBHController::isSettled(){
    return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativeVelTBHController::reset(){
    LOG_INFO_S("IterativeVelPIDController: Reset");

    error = 0;
    prevError = 0;
    tbh = 0;
    output = 0;
    settledUtil->reset();
}

void IterativeVelTBHController::flipDisable(){
    flipDisable(!controllerIsDisabled);
}

void IterativeVelTBHController::flipDisable(const bool iIsDisabled){
    LOG_INFO("IterativeVelPIDController: flipDisable " + std::to_string(iIsDisabled));
    controllerIsDisabled = iIsDisabled;
}

bool IterativeVelTBHController::isDisabled() const{
    return controllerIsDisabled;
}

void IterativeVelTBHController::setGains(const double iGain){
    gain = iGain;
}

double IterativeVelTBHController::getGains() const{
    return gain;
}

void IterativeVelTBHController::setTicksPerRev(double iTPR){
    velMath->setTicksPerRev(iTPR);
}

okapi::QAngularSpeed IterativeVelTBHController::getVel() const{
    return velMath->getVelocity();
}

okapi::QTime IterativeVelTBHController::getSampleTime() const{
    return sampleTime;
}

}
