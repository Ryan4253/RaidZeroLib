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
#include <iostream>
#include "Geometry/Pose.hpp"
#include "lib4253/Utility/Units.hpp"

namespace lib4253{
using namespace okapi;

/**
 * @brief Trajectory point structure
 *
 */
struct TrajectoryPoint{
    Pose position{0_m, 0_m, 0_rad};
    QLength leftDist{0_m}, rightDist{0_m};
    QSpeed leftVel{0_mps}, rightVel{0_mps};
    QAcceleration leftAccel{0_mps2}, rightAccel{0_mps2};

    /**
     * @brief Construct a new Trajectory Point object
     * 
     */
    TrajectoryPoint() = default;

    /**
     * @brief Construct a new Trajectory Point object
     * 
     * @param iLeftDist left distance
     * @param iRightDist right distance
     * @param iLeftVel left velocity
     * @param iRightVel right velocity
     * @param leftAccel left aceleration
     * @param rightAccel right acceleration
     * @param iPosition global coordinate
     */
    TrajectoryPoint(const QLength iLeftDist, const QLength iRightDist,
                    const QSpeed iLeftVel, const QSpeed iRightVel,
                    const QAcceleration leftAccel, const QAcceleration rightAccel,
                    const Pose& iPosition = Pose{0_m, 0_m, 0_rad});

    /**
     * @brief Destroy the Trajectory Point object
     * 
     */
    ~TrajectoryPoint() = default;

    friend std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt);
};

/**
 * @brief Prints the information of a trajectory point to an existing ostream
 * 
 * @param os the output stream to output to
 * @param pt the trajectory point to print
 * @return std::ostream& an ongoing output stream
 */
std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt);

/**
 * @brief class which stores a trajectory - a series of target position, velocity and acceleration
 *        which allows our robot to follow a predetermined path. The paths are pre-generated on 
 *        our computer and then deployed into our code.
 */
class Trajectory{
    private:
    std::vector<TrajectoryPoint> path;

    public:
    /**
     * @brief Construct a new Trajectory object
     * 
     */
    Trajectory() = default;
    
    /**
     * @brief Destroy the Trajectory object
     * 
     */
    ~Trajectory() = default;

    /**
     * @brief Construct a new Trajectory object
     * 
     * @param iPath the trajectory
     */
    Trajectory(const std::initializer_list<TrajectoryPoint>& iPath);
    
    /**
     * @brief gets the trajectory information at a given point of the trajectory
     * 
     * @param index the index (time) to check
     * @return TrajectoryPoint the trajectory point at the given time
     */
    TrajectoryPoint operator[](int index) const;

    /**
     * @brief returns the size of the trajectory. Every index stores 10ms of target kinematics
     * 
     * @return int the size of the trajectory
     */
    int size() const;
};

}