/**
 * @file PurePursuit.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Pure pursuit controller - https://media.istockphoto.com/vectors/kitten-guiding-puppy-with-bone-vector-id1283672337?k=6&m=1283672337&s=612x612&w=0&h=yi3d4m_LMXtYzhZZamDOLE800JJ69VL1TVd1fGbFWSo=
 * @version 1.0
 * @date 2021-07-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Trajectory/Spline/PurePursuitPath.hpp"
#include "lib4253/Trajectory/Geometry/Pose2D.hpp"
#include "lib4253/Chassis/Device/Chassis.hpp"
#include "lib4253/Chassis/Device/Odometry.hpp"
#include <vector>


namespace lib4253 {

/**
 * @brief Pure pursuit class
 *
 */
class AdaptivePurePursuitController{
    public:

    /**
     * @brief Constructor
     * 
     * @param iChassis - the chassis instance
     * @param iOdometry - the odometry instance
     * @param iLookAheadDist - lookahead distance
     */
    AdaptivePurePursuitController(std::shared_ptr<Chassis> iChassis, std::shared_ptr<Odometry> iOdometry, okapi::QLength iLookAheadDist);

    /**
     * @brief follows a given path
     * 
     * @param path - the path to follow
     */
    void followPath(const PurePursuitPath& path);

    /**
     * @brief sets instance's lookahead distance
     * 
     * @param iLookAhead - new lookahead dist
     */
    void setLookAhead(okapi::QLength iLookAhead);

    /**
     * @brief sets instance's lookahead distance, returns itself for function chaining
     * 
     * @param iLookAhead - new lookahead dist
     * @return the instance of the adaptive pure pursuit controller
     */
    AdaptivePurePursuitController& withLookAhead(okapi::QLength iLookAhead);

    private:

    /** 
     * @brief Initializes pure pursuit controller
     *
     */
    void initialize();

    /**
     * @brief Calculates closest point on path to generate desired motor velocities
     *
     */
    void updateClosestPoint();

    // TO DO: optimize    

    /**
      * @brief Calculates the look ahead point (aka. the next target point)
      *
      */
    void calculateLookAheadPoint();

    /**
     * @brief Calculates curvature of the path to assist with turning
     *
     */
    void calculateCurvature();

    /**
      * @brief Calculates power for drive
      *
      */
    void calculateVelocity();

    /**
     * @brief Checks if drive has stopped
     * 
     * @return true - drive stopped
     * @return false - drive in motion
     */
    bool isSettled();


    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<Odometry> odometry;
    okapi::QLength lookAheadDist;

    PurePursuitPath path;

    Pose2D currentPos;

    int closestPoint;
    int prevClosestPoint;

    int prevLookAheadPoint;
    Point2D lookAheadPoint;

    okapi::QCurvature curvature;

    okapi::QSpeed vel;
    okapi::QAngularSpeed angularVel;

    bool settle;

};
}