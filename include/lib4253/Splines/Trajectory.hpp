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
#include "main.h"

/**
 * @brief Trajectory point structure
 *
 */
struct TrajectoryPoint{
    /**
     * @brief Stores the velocity
     *
     */
    double velocity;

    /**
     * @brief Stores the acceleration
     *
     */
    double acceleration;


    /**
     * @brief Construct a new Trajectory Point object
     *
     * @param v velocity
     * @param a acceleration
     */
    TrajectoryPoint(double v, double a);
};

  /**
   * @brief Trajectory class
   *
   */
class Trajectory{
    private:
    std::vector<TrajectoryPoint> left;
    std::vector<TrajectoryPoint> right;

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
    Trajectory(std::vector<TrajectoryPoint> l, std::vector<TrajectoryPoint> r);

    /**
     * @brief Gets size of the trajectory
     *
     * @return size
     */
    int getSize();

    /**
     * @brief Gets the velocity and acceleration at the given index
     *
     * @param index index
     * @return velocity and acceleration at index
     */
    std::pair<TrajectoryPoint, TrajectoryPoint> getKinematics(int index);

};
