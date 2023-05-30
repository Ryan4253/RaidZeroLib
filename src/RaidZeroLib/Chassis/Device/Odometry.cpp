
#include "Odometry.hpp"
namespace lib4253{

OneEncoderImuOdometry::OneEncoderImuOdometry(const std::shared_ptr<ContinuousRotarySensor>& iSide,
                                             const std::shared_ptr<IMU>& Imu,
                                             const ChassisScales& iScales,
                                             const TimeUtil& iTimeUtil, 
                                             const std::shared_ptr<Logger>& iLogger):chassisScales(iScales){
    side = std::move(iSide);
    imu = std::move(Imu);
    logger = std::move(iLogger);
    rate = std::move(iTimeUtil.getRate());
    timer = std::move(iTimeUtil.getTimer());                                     
}

void OneEncoderImuOdometry::setScales(const ChassisScales& ichassisScales){
    chassisScales = ichassisScales;
}

void OneEncoderImuOdometry::step(){
    const auto deltaT = timer->getDt();
    if(deltaT.getValue() != 0){
        newTicks[0] = side->get();
        newTicks[1] = Math::angleWrap180(-imu->get()*degree).convert(degree);
        tickDiff = newTicks - lastTicks;
        tickDiff[1] = Math::angleWrap180(tickDiff[1]*degree).convert(degree);
        lastTicks = newTicks;

        const auto newState = odomMathStep(tickDiff, deltaT);

        state.x += newState.x;
        state.y += newState.y;
        state.theta += newState.theta;
    }
}

OdomState OneEncoderImuOdometry::getState(const StateMode &imode) const{
    if(imode == StateMode::FRAME_TRANSFORMATION){
        return state;
    }
    else{
        return OdomState{state.y, state.x, state.theta};
    }
}

void OneEncoderImuOdometry::setState(const OdomState &istate, const StateMode &imode) {
    LOG_DEBUG("State set to: " + istate.str());
    if(imode == StateMode::FRAME_TRANSFORMATION){
        state = istate;
    }
    else{
        state = OdomState{istate.y, istate.x, istate.theta};
    }
}

ChassisScales OneEncoderImuOdometry::getScales(){
    return chassisScales;
}

OdomState OneEncoderImuOdometry::odomMathStep(const std::valarray<double> &itickDiff, const QTime &ideltaT) const{
    /*
        if (itickDiff.size() < 2) {
        LOG_ERROR_S("OneEncoderIMUOdometry: itickDiff did not have at least three elements.");
        return OdomState{};
    }

    for (auto &&elem : itickDiff) {
        if (std::abs(elem) > maximumTickDiff) {
            LOG_ERROR("OneEncoderIMUOdometry: A tick diff (" + std::to_string(elem) +
                        ") was greater than the maximum allowable diff (" +
                        std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
            return OdomState{};
        }
    }

    const double deltaS = itickDiff[0] / chassisScales.straight;
    double deltaTheta = (itickDiff[1] * degree).convert(radian);
    double localOffX, localOffY;

    if (deltaTheta == 0) {
        localOffX = 0;
        localOffY = deltaS;
    } else {
        localOffX = 2 * std::sin(deltaTheta / 2) *
                    (deltaM / deltaTheta + chassisScales.middleWheelDistance.convert(meter) * 2);
        localOffY = 2 * std::sin(deltaTheta / 2) *
                    (deltaS / deltaTheta + chassisScales.wheelTrack.convert(meter));
    }

    double avgA = state.theta.convert(radian) + (deltaTheta / 2);

    double polarR = std::sqrt((localOffX * localOffX) + (localOffY * localOffY));
    double polarA = std::atan2(localOffY, localOffX) - avgA;

    double dX = std::sin(polarA) * polarR;
    double dY = std::cos(polarA) * polarR;

    if (isnan(dX)) {
        dX = 0;
    }

    if (isnan(dY)) {
        dY = 0;
    }

    if (isnan(deltaTheta)) {
        deltaTheta = 0;
    }

    return OdomState{dX * meter, dY * meter, deltaTheta * radian};
    */
   return OdomState{0*meter, 0*meter, 0*radian};
}

std::shared_ptr<ReadOnlyChassisModel> OneEncoderImuOdometry::getModel(){
    return nullptr;
}

TwoEncoderImuOdometry::TwoEncoderImuOdometry(const std::shared_ptr<ContinuousRotarySensor>& iSide,
                                             const std::shared_ptr<ContinuousRotarySensor>& iMiddle,
                                             const std::shared_ptr<IMU>& Imu,
                                             const ChassisScales& iScales,
                                             const TimeUtil& iTimeUtil, 
                                             const std::shared_ptr<Logger>& iLogger) 
    : OneEncoderImuOdometry(std::move(iSide), std::move(Imu), iScales, iTimeUtil, std::move(iLogger)){

    middle = std::move(iMiddle);
}




void TwoEncoderImuOdometry::step(){
    const auto deltaT = timer->getDt();
    if(deltaT.getValue() != 0){
        newTicks[0] = side->get();
        newTicks[1] = middle->get();
        newTicks[2] = Math::angleWrap180(-imu->get()*degree).convert(degree);
        tickDiff = newTicks - lastTicks;
        tickDiff[2] = Math::angleWrap180(tickDiff[2]*degree).convert(degree);
        lastTicks = newTicks;

        const auto newState = odomMathStep(tickDiff, deltaT);

        state.x += newState.x;
        state.y += newState.y;
        state.theta += newState.theta;
    }
}

OdomState TwoEncoderImuOdometry::odomMathStep(const std::valarray<double> &itickDiff,
                                              const QTime &ideltaT) const{
    if (itickDiff.size() < 3) {
        LOG_ERROR_S("TwoEncoderIMUOdometry: itickDiff did not have at least three elements.");
        return OdomState{};
    }

    for (auto &&elem : itickDiff) {
        if (std::abs(elem) > maximumTickDiff) {
            LOG_ERROR("TwoEncoderIMUOdometry: A tick diff (" + std::to_string(elem) +
                        ") was greater than the maximum allowable diff (" +
                        std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
            return OdomState{};
        }
    }

    const double deltaS = itickDiff[0] / chassisScales.straight;
    double deltaTheta = (itickDiff[2] * degree).convert(radian);
    const auto deltaM = static_cast<const double>(
        itickDiff[2] / chassisScales.middle -
        ((deltaTheta / 2_pi) * 1_pi * chassisScales.middleWheelDistance.convert(meter) * 2));

    double localOffX, localOffY;

    if (deltaTheta == 0) {
        localOffX = deltaM;
        localOffY = deltaS;
    } else {
        localOffX = 2 * std::sin(deltaTheta / 2) *
                    (deltaM / deltaTheta + chassisScales.middleWheelDistance.convert(meter) * 2);
        localOffY = 2 * std::sin(deltaTheta / 2) *
                    (deltaS / deltaTheta + chassisScales.wheelTrack.convert(meter));
    }

    double avgA = state.theta.convert(radian) + (deltaTheta / 2);

    double polarR = std::sqrt((localOffX * localOffX) + (localOffY * localOffY));
    double polarA = std::atan2(localOffY, localOffX) - avgA;

    double dX = std::sin(polarA) * polarR;
    double dY = std::cos(polarA) * polarR;

    if (isnan(dX)) {
        dX = 0;
    }

    if (isnan(dY)) {
        dY = 0;
    }

    if (isnan(deltaTheta)) {
        deltaTheta = 0;
    }

    return OdomState{dX * meter, dY * meter, deltaTheta * radian};
}



}