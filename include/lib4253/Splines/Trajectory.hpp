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
class Trajectory{
    private:
    std::vector<TrajectoryPoint> traj;

    public:
    /**
     * @brief Construct a new Trajectory object
     *
     */
    Trajectory() = default;

    /**
     * @brief Construct a new Trajectory object
     *
     * @param l left trajectory points
     * @param r right trajectory points
     */
    Trajectory(const std::vector<TrajectoryPoint>& path);

    /**
     * @brief Gets size of the trajectory
     *
     * @return size
     */
    int getSize() const;

    /**
     * @brief Gets the velocity and acceleration at the given index
     *
     * @param index index
     * @return velocity and acceleration at index
     */
    TrajectoryPoint getKinematics(int index) const;
};
}