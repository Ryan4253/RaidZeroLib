/**
 * @file Trajectory.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Trajectories
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <vector>
#include "Geometry/Pose2D.hpp"
#include "lib4253/Utility/Units.hpp"

namespace lib4253{

/**
 * @brief Trajectory point structure
 *
 */
struct TrajectoryPoint{
    /**
     * @brief Stores the velocity
     *
     */
    Pose2D position;

    /**
     * @brief Stores the inear velocity
     *
     */
    okapi::QSpeed linVelocity;

    /**
     * @brief Stores the linearacceleration
     *
     */
    okapi::QAcceleration linAcceleration;

    /**
     * @brief Stores the angular velocity
     *
     */
    okapi::QAngularSpeed angVelocity;

    /**
     * @brief Stores the angular acceleration
     *
     */
    okapi::QAngularAcceleration angAcceleration;

    /**
     * @brief Construct a new Trajectory Point object
     *
     * @param v velocity
     * @param a acceleration
     */
    TrajectoryPoint(const Pose2D& p, 
                    const okapi::QSpeed& linV, 
                    const okapi::QAcceleration& linA, 
                    const okapi::QAngularSpeed& angV, 
                    const okapi::QAngularAcceleration& angA);
};

    /**
     * @brief Trajectory class
     *
     */
typedef std::vector<TrajectoryPoint> Trajectory;
}