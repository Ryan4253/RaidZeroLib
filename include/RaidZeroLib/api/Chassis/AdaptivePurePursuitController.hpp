#pragma once
#include "RaidZeroLib/api/Control/Feedforward/FeedforwardController.hpp"
#include "RaidZeroLib/api/Filter/SlewRate.hpp"
#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include "RaidZeroLib/api/Utility/CrossPlatformThread.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include <memory>

namespace rz {
using namespace okapi;

/**
 * The AdaptivePurePurePursuitController is a controller that drives the robot along a path using the pure pursuit
 * algorithn. In each iteration, the controller computes the lookahead point on the path that is one lookahead distance
 * away from the robot. The robot then drives towards the point at constant curvature given the target velocity. This
 * process is repeated  until the end of the path is reached.
 *
 */
class AdaptivePurePursuitController {
    public:
    /**
     * Gains that characterize the behavior of the controller.
     *
     */
    struct Gains {
        QSpeed maxVelocity{0.0};
        QAcceleration maxAcceleration{0.0};
        QAngularSpeed maxTurnVelocity{0.0};
        QLength lookAhead{0.0};

        /**
         * Sets all gains to zero.
         */
        Gains() = default;

        /**
         * Destructor.
         */
        ~Gains() = default;

        /**
         * Sets all gains to the given values.
         *
         * @param maxVelocity Maximum velocity
         * @param maxAcceleration Maximum acceleration
         * @param maxTurnVelocity Maximum turn velocity
         * @param lookAhead Lookahead distance
         */
        Gains(QSpeed maxVelocity, QAcceleration maxAcceleration, QAngularSpeed maxTurnVelocity, QLength lookAhead);

        /**
         * Checks if two Gains objects are equal.
         *
         * @param rhs The Gains object to compare to
         *
         * @return True if the two objects are equal, false otherwise
         */
        bool operator==(const Gains& rhs) const;

        /**
         * Checks if two Gains objects are not equal.
         *
         * @param rhs The Gains object to compare to
         *
         * @return True if the two objects are not equal, false otherwise
         */
        bool operator!=(const Gains& rhs) const;
    };

    /**
     * Constructs a new AdaptivePurePursuitController.
     *
     * @param chassis The chassis to control
     * @param gains The gains to use
     * @param leftController The left side feedforward velocity controller. If this is null, the controller will use
     * vex's internal velocity control
     * @param rightController The right side feedforward velocity controller. If this is null, the controller will use
     * vex's internal velocity control
     * @param timeUtil The time utility to use
     */
    AdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& chassis, const Gains& gains,
                                  std::unique_ptr<FeedforwardController<QLength>> leftController = nullptr,
                                  std::unique_ptr<FeedforwardController<QLength>> rightController = nullptr,
                                  const TimeUtil& timeUtil);

    /**
     * Follows the given path. The path is followed until the end is reached or the timeout is exceeded.
     *
     * @param path The path to follow
     * @param timeout The maximum time allowed to follow the path. This is to prevent the robot from getting stuck
     * @param isReversed Whether or not to follow the path in reverse
     */
    void followPath(const DiscretePath& path, QTime timeout = 2_min, bool isReversed = false);

    /**
     * Stops the chassis and any running movements.
     *
     */
    void stop();

    /**
     * Blocks the current movement until the movement is complete.
     *
     */
    void waitUntilSettled();

    private:
    /**
     * Computes the lookahead point on the path.
     *
     * @param path The path to get the lookahead point from
     * @param minIndex The minimum index to start searching from. This is updated to the index the lookahead point was
     * found at
     * @param point Robot position
     * @param radius Radius of the lookahead circle
     *
     * @return The lookahead point, or std::nullopt if no point was found
     */
    std::optional<Point> getLookaheadPoint(const DiscretePath& path, double& minIndex, const Point& point,
                                           QLength radius);

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