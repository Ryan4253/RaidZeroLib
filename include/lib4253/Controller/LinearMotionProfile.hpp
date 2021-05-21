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
#include "main.h"

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
    LinearMotionProfileController(double maxA, double maxVel);

  public:
    /**
     * @brief Set maximum velocity and acceleration
     *
     * @param maxV max velocity
     * @param maxA max acceleration
     */
    void setKinematics(double maxV, double maxA);

    /**
     * @brief Sets desired distance for the controller
     *
     * @param d desired distance
     */
    void setDistance(double d);

    /**
     * @brief Gets velocity at specified time step
     *
     * @param t time step
     * @return velocity at t
     */
    double getVelocityTime(double t);

    /**
     * @brief Gets velocity at specified distance
     *
     * @param d distance
     * @return velocity at d
     */
    double getVelocityDist(double d);

    /**
     * @brief Gets predicted execution time
     *
     * @return total time
     */
    double getTotalTime();

};

class TrapezoidalProfileController:LinearMotionProfileController{

};

class SCurveMotionProfileController:LinearMotionProfileController{

};
