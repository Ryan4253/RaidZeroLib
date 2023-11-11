#pragma once
#include "RaidZeroLib/api/Control/Feedforward/FeedforwardController.hpp"
#include "RaidZeroLib/api/Filter/SlewRate.hpp"
#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include "RaidZeroLib/api/Utility/CrossPlatformThread.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <memory>

namespace rz {
using namespace okapi;

class AdaptivePurePursuitController {
    public:
    struct Gains {
        QSpeed maxVelocity{0.0};
        QAcceleration maxAcceleration{0.0};
        QAngularSpeed maxTurnVelocity{0.0};
        QLength lookAhead{0.0};

        Gains() = default;
        ~Gains() = default;
        Gains(QSpeed maxVelocity, QAcceleration maxAcceleration, QAngularSpeed maxTurnVelocity, QLength lookAhead);
        bool operator==(const Gains& rhs) const;
        bool operator!=(const Gains& rhs) const;
    };

    AdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& chassis, const Gains& gains,
                                  std::unique_ptr<FeedforwardController<QLength>> leftController = nullptr,
                                  std::unique_ptr<FeedforwardController<QLength>> rightController = nullptr,
                                  const TimeUtil& timeUtil = okapi::TimeUtilFactory::createDefault());

    void followPath(DiscretePath& path, QTime timeout = 2_min, bool isReversed = false);

    void stop();

    void waitUntilSettled();

    private:
    std::optional<Point> getLookaheadPoint(DiscretePath& path, double& minIndex, const Point& point, QLength radius);
    void generateKinematics();

    std::shared_ptr<OdomChassisController> chassis;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;

    Gains gains;

    TimeUtil timeUtil;

    std::unique_ptr<FeedforwardController<QLength>> leftController;
    std::unique_ptr<FeedforwardController<QLength>> rightController;

    CrossPlatformThread task;
};

} // namespace rz