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
    /**
     * @brief Converts degrees to radians
     * 
     * @param deg angle in degrees
     * @return converted angle in radians
     */
    double degToRad(double deg); 

    /**
     * @brief Converts radians to degrees
     * 
     * @param rad angle in radians
     * @return converted angle in degrees
     */
    double radToDeg(double rad); 

    // conversion between inches and encoder ticks. For the first function,
    // The wheel size is defaulted to be to 2.75" and 360 ticks per rotation.
    // Second functions allows the user to specify the whele size and ticks per inch
    /**
     * @brief Converts encoder ticks to inches
     *        Assumes that the wheel size is 2.75" and 360 ticks per rotation.
     * 
     * @param tick encoder ticks
     * @return inches travelled
     */
    double tickToInch(double tick);

    /**
     * @brief Converts encoder ticks to inches
     * 
     * @param tick encoder ticks
     * @param rad wheel radius
     * @param ticksPerRotation amount of encoder ticks per wheel rotation
     * @return inches travelled
     */
    double tickToInch(double tick, double rad, double ticksPerRotation);

    /**
     * @brief Converts inches to encoder ticks
     *        Assumes that the wheel size is 2.75" and 360 ticks per rotation.
     * 
     * @param inch inches travelled
     * @return amount of wheel ticks in the distance travelled 
     */
    double inchToTick(double inch);

    /**
     * @brief Converts inches to encoder ticks
     * 
     * @param inch inches travelled 
     * @param rad wheel radius
     * @param ticksPerRotation amount of encoder ticks per wheel rotation
     * @return amount of wheel ticks in the distance travelled 
     */
    double inchToTick(double inch, double rad, double ticksPerRotation);

    /**
     * @brief Converts encoder ticks to the amount of degrees the robot rotates
     * 
     * @param tick encoder ticks
     * @param scale chassis scale 
     * @param ticksPerRotation amount of ticks recorded per rotation
     * @return number of degrees travelled
     */
    double tickToDeg(const double& tick, const okapi::ChassisScales& scale);

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
    double wrapAngle360(double angle);

    /**
     * @brief Limits angles between the range [-180, 180]
     * 
     * @param angle original angle
     * @return limited angle
     */
    double wrapAngle180(double angle);

    /**
     * @brief Limits angles between the range [-90, 90]
     * 
     * @param angle original angle
     * @return limited angle
     */
    double wrapAngle90(double angle);

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
     * @brief Limits the input between the range [mn, mx]
     * 
     * @param val value to be limited
     * @param mn minimum range
     * @param mx maximum range
     * @return limited value
     */
    double clamp(double val, double mn, double mx);

    /**
     * @brief calculates sinc(x)
     * 
     * @param x input
     * @return sinc(x)
     */
    double sinc(double x);
};
}