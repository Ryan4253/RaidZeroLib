/**
 * @file PurePursuit.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Pure pursuit controller - https://media.istockphoto.com/vectors/kitten-guiding-puppy-with-bone-vector-id1283672337?k=6&m=1283672337&s=612x612&w=0&h=yi3d4m_LMXtYzhZZamDOLE800JJ69VL1TVd1fGbFWSo=
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "main.h"
#include "lib4253/Splines/SimplePath.hpp"
namespace lib4253{


/**
 * @brief Pure pursuit class
 *
 */
class PurePursuitFollower{
    private:
    SimplePath path;
    std::vector<double> velocity;
    double maxAcceleration = 0, maxVelocity = 0, trackWidth, radius, kT;
    int prevClosestPoint = 0, closestPoint = 0;
    int prevLookAheadPoint = 0; Point2D lookAheadPoint;
    double curvature;
    bool settled = false;
    
    /**
     * @brief Generates the desired velocity
     *
     */
    void generateVelocity();
    
    /**
     * @brief Calculates closest point on path to generate desired motor velocities
     *
     * @param currentPos
     */
    void calcClosestPoint(Pose2D currentPos);
    
    /**
     * @brief Calculates the look ahead point (aka. the next target point)
     *
     * @param currentPos
     */
    void calcLookAheadPoint(Pose2D currentPos);
    
    /**
     * @brief Calculates curvature of the path to assist with turning
     *
     * @param currentPos
     */
    void calcCurvature(Pose2D currentPos);
    
    /**
     * @brief Calculates power for drive
     *
     * @param currentPos current position of robot
     * @return motor velocities for each side
     */
    std::pair<double, double> calcPower(Pose2D currentPos);
    
    public:
    /**
     * @brief Set the Path for pure pursuit controller
     *
     * @param path the path to be followed
     */
    void setPath(SimplePath path);
    
    /**
     * @brief Initializes pure pursuit controller
     *
     */
    void initialize();
    
    /**
     * @brief
     *
     * @param CurrentPos
     * @return std::pair<double, double>
     */
    std::pair<double, double> step(Pose2D CurrentPos);
    
    /**
     * @brief Sets turn gain
     *
     * @param k turn gain
     */
    void setTurnGain(double k);
    
    /**
     * @brief Sets maximum acceleration and velocity
     *
     * @param v maximum velocity
     * @param a maximum acceleration
     */
    void setKinematics(double v, double a);
    
    /**
     * @brief Sets track width of drive
     *
     * @param size track width in inches
     */
    void setTrackWidth(double size);
    
    /**
     * @brief Sets radius of look-ahead distance
     *
     * @param dist radius of the look-ahead distance
     */
    void setLookAhead(double dist);
    
    /**
     * @brief Checks if drive has stopped
     * 
     * @return true - drive stopped
     * @return false - drive in motion
     */
    bool isSettled();
};
}