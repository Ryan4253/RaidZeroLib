#pragma once
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Utility/Units.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"
#include <tuple>
#include <atomic>
namespace lib4253{
using namespace okapi;

class OneEncoderImuOdometry : public Odometry{
    public:
    OneEncoderImuOdometry(const std::shared_ptr<ContinuousRotarySensor>& iSide,
                          const std::shared_ptr<IMU>& Imu,
                          const ChassisScales& iScales,
                          const TimeUtil& iTimeUtil = TimeUtilFactory::createDefault(),
                          const std::shared_ptr<Logger>& iLogger = Logger::getDefaultLogger());

    virtual ~OneEncoderImuOdometry() = default;

    void setScales(const ChassisScales& ichassisScales) override;

    void step() override;

    OdomState getState(const StateMode &imode = StateMode::FRAME_TRANSFORMATION) const override;

    void setState(const OdomState& istate,
                  const StateMode& imode = StateMode::FRAME_TRANSFORMATION) override;

    ChassisScales getScales() override;
    
    protected:
    std::shared_ptr<Logger> logger;
    std::unique_ptr<AbstractRate> rate;
    std::unique_ptr<AbstractTimer> timer;
    std::shared_ptr<ContinuousRotarySensor> side;
    std::shared_ptr<IMU> imu;
    ChassisScales chassisScales;
    OdomState state;
    std::valarray<std::int32_t> newTicks{0, 0, 0}, tickDiff{0, 0, 0}, lastTicks{0, 0, 0};
    const std::int32_t maximumTickDiff{1000};

    virtual OdomState odomMathStep(const std::valarray<std::int32_t> &itickDiff,
                                   const QTime &ideltaT) const;

    private:
    std::shared_ptr<ReadOnlyChassisModel> getModel() override;

};

class TwoEncoderImuOdometry : public OneEncoderImuOdometry{
    public:
    TwoEncoderImuOdometry(const std::shared_ptr<ContinuousRotarySensor>& iSide,
                          const std::shared_ptr<ContinuousRotarySensor>& iMiddle,
                          const std::shared_ptr<IMU>& Imu,
                          const ChassisScales& iScales,
                          const TimeUtil& iTimeUtil = TimeUtilFactory::createDefault(), 
                          const std::shared_ptr<Logger>& iLogger = Logger::getDefaultLogger());

    virtual ~TwoEncoderImuOdometry() = default;

    void step() override;

    protected:
    std::shared_ptr<ContinuousRotarySensor> middle;

    virtual OdomState odomMathStep(const std::valarray<std::int32_t> &itickDiff,
                                   const QTime &ideltaT) const override;
};

}