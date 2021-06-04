/**
 * @file LinearMotionProfile.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Generates and executes linear motion profiles
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <math.h>
namespace lib4253{

/**
 * @brief Linear motion profile controller class
 *
 */
class LinearMotionProfileController{
    private:
    double maxAcc, decel, maxVel;
    double tAcc, tCruise;
    double dAcc, dCruise;
    double dist;

    //double startVel, endVel

    /**
     * @brief Construct a new Linear Motion Profile Controller object
     *
     */
    LinearMotionProfileController();

    /**
     * @brief Construct a new Linear Motion Profile Controller object
     *
     * @param a maximum acceleration
     * @param maxV maximum velocity
     */
    LinearMotionProfileController(const double& maxA, const double& maxVel);

    /**
     * @brief Destroys the Linear Motion Profile Controller object
     * 
     */
    ~LinearMotionProfileController() = default;

    public:
    /**
     * @brief Set maximum velocity and acceleration
     *
     * @param maxV max velocity
     * @param maxA max acceleration
     */
    void setKinematics(const double& maxV, const double& maxA);

    /**
     * @brief Sets desired distance for the controller
     *
     * @param d desired distance
     */
    void setDistance(const double& d);

    /**
     * @brief Gets velocity at specified time step
     *
     * @param t time step
     * @return velocity at t
     */
    double getVelocityTime(const double& t) const;

    /**
     * @brief Gets velocity at specified distance
     *
     * @param d distance
     * @return velocity at d
     */
    double getVelocityDist(const double& d) const;

    /**
     * @brief Gets predicted execution time
     *
     * @return total time
     */
    double getTotalTime() const;
};

class TrapezoidalProfileController : LinearMotionProfileController{

};

class SCurveMotionProfileController : LinearMotionProfileController{

};
}