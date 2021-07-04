#pragma once
#include "lib4253/Splines/Geometry/Point2D.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "okapi/api/units/QLength.hpp"
#include <math.h>
#include <memory>

namespace lib4253{
/*
* Math.hpp
*
* This file contains some useful math functions used across the code, such as
* unit conversions and limiting output
*/

namespace Math{

    // conversion between inches and encoder ticks. For the first function,
    // The wheel size is defaulted to be to 2.75" and 360 ticks per rotation.
    // Second functions allows the user to specify the whele size and ticks per inch

    /**
     * @brief Converts encoder ticks to inches
     * 
     * @param tick encoder ticks
     * @param rad wheel radius
     * @param ticksPerRotation amount of encoder ticks per wheel rotation
     * @return inches travelled
     */
    okapi::QLength angleToArcLength(const okapi::QAngle& angle, const okapi::QLength& rad = 1.375 * okapi::inch);

    /**
     * @brief Converts inches to encoder ticks
     * 
     * @param inch inches travelled 
     * @param rad wheel radius
     * @param ticksPerRotation amount of encoder ticks per wheel rotation
     * @return amount of wheel ticks in the distance travelled 
     */
    okapi::QAngle arcLengthToAngle(const okapi::QLength& dist, const okapi::QLength& rad = 1.375 * okapi::inch);

    /**
     * @brief Converts encoder ticks to the amount of degrees the robot rotates
     * 
     * @param tick encoder ticks
     * @param scale chassis scale 
     * @param ticksPerRotation amount of ticks recorded per rotation
     * @return number of degrees travelled
     */
    okapi::QAngle angleToYaw(const okapi::QAngle& angle, const okapi::ChassisScales& scale);

    /**
     * @brief Scales the input power cubically, used in driver control for more precise control (theoretically)
     * 
     * @param power input power
     * @return cubically scalled power
     */
    double cubicControl(double power);

    /**
     * @brief Limits angles between the range [0, 360]
     * 
     * @param angle original angle
     * @return limited angle
     */
    okapi::QAngle angleWrap360(const okapi::QAngle& angle);

    /**
     * @brief Limits angles between the range [-180, 180]
     * 
     * @param angle original angle
     * @return limited angle
     */
    okapi::QAngle angleWrap180(const okapi::QAngle& angle);

    /**
     * @brief Limits angles between the range [-90, 90]
     * 
     * @param angle original angle
     * @return limited angle
     */
    okapi::QAngle angleWrap90(const okapi::QAngle& angle);

    // conversion  between linear velocity and motor rpm
    /**
     * @brief Converts linear velocity to motor RPM
     * 
     * @param linVel linear velocity
     * @param GearRatio gear ratio
     * @param radius wheel radius
     * @return motor RPM
     */
    double linearVelToRPM(double linVel, double GearRatio, double radius);

    /**
     * @brief Converts motor RPM to linear velocity
     * 
     * @param angVel motor RPM
     * @param gearRatio gear ratio
     * @param radius wheel radius
     * @return linear velocity
     */
    double RPMToLinearVel(double angVel, double gearRatio, double radius);

    /**
     * @brief calculates sinc(x)
     * 
     * @param x input
     * @return sinc(x)
     */
    double sinc(double x);
};
}