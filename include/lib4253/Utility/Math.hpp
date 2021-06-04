#pragma once
#include "lib4253/Splines/Point2D.hpp"
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
    double degToRad(double deg); // converts degree to radian
    double radToDeg(double rad); // converts radian to degree

    // conversion between inches and encoder ticks. For the first function,
    // The wheel size is defaulted to be to 2.75" and 360 ticks per rotation.
    // Second functions allows the user to specify the whele size and ticks per inch
    double tickToInch(double tick);
    double tickToInch(double tick, double rad, double ticksPerRotation);

    double inchToTick(double inch);
    double inchToTick(double tick, double rad, double ticksPerRotation);

    double tickToDeg(const double& tick, const std::shared_ptr<okapi::ChassisScales>& scale, const double& ticksPerRotation);

    // scales the pwn power cubically, used for driver control for more precise control
    double cubicControl(double power);

    // limit angles between the range [0, 360], [-180, 180] and [-90, 90]
    double wrapAngle360(double angle);
    double wrapAngle180(double angle);
    double wrapAngle90(double angle);

    // conversion  between linear velocity and motor rpm
    double linearVelToRPM(double linVel, double GearRatio, double radius);
    double RPMToLinearVel(double angVel, double gearRatio, double radius);

    // limits the input between the range [mn, mx]
    double clamp(double val, double mn, double mx);
};
}