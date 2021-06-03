#pragma once
#include "main.h"
#include "lib4253/Utility/Math.hpp"
namespace lib4253{

/*
* CustomOdometry.hpp
*
* This file contains the declaration for
* the robot's odometry system. It is a mean to use sensor feedback from encoders
* and imus to estimate the robot's absolute position on the field, therefore
* allowing a lot of new possibilities such as error autocorrection or path following
*
* As there are many different methods to implement odometry, The code consists of
* multiple classes. The base class is CustomOdometry, which provides general methods
* (ex. getter, setter) and virtual methods for its subclasses
*
* Each of the other classes are subclasses of CustomOdometry, where they all use
* different sensor and have slightly different
*/

class CustomOdometry{
    protected:
    Pose2D globalPos; // the global position of the robot

    // updates the position of the robot, set as pure virtual so each odometry
    // archetype can implement their own updatePos function
    virtual void updatePos() = 0;

    public:
    CustomOdometry(); // constructor
    virtual void withDimensions(std::tuple<double, double> dimension); // sets the dimension for 2 wheel odometry
    virtual void withDimensions(std::tuple<double, double, double> dimension); // sets the dimension for 2 wheel odometry

    Pose2D getPos(); // return current position as a struct
    double getX(); // return x position as a double, units in inches
    okapi::QLength getQX(); // return x position as type QLength
    double getY(); // return y position as a double, in unit inches
    okapi::QLength getQY(); // return y position in type QLength
    double getAngleDeg(); // return angle in degrees
    double getAngleRad(); // return angle in radians

    virtual double getEncoderLeft(); // gets the left encoder reading
    virtual double getEncoderRight(); // gets the right encoder reading
    virtual double getEncoderMid(); // gets the mid encoder reading

    void setPos(Pose2D newPos); // sets the current position
    void setX(double x); // sets the x position of the robot
    void setX(okapi::QLength inch); // sets the x position of the robot
    void setY(double y); // sets the y position of the robot
    void setY(okapi::QLength inch); // sets the y position of the robot
    void setAngleDeg(double theta); // sets the angle of the robot in degrees
    void setAngleRad(double theta); // sets the angle of the robot in radians

    void displayPosition(); // outputs the x, y, angle of the robot on the robot screen and the console

    void resetState(); // sets the robot position to {0, 0, 0}
    virtual void resetSensors() = 0; // resets robot sensors. set as pure virtual so each subclass can reset their individual sensors
    void reset(); // resets the position and the sensor
    static void odomTask(void *ptr); // trampoline for the task to run
};

// Odometry using 3 encoders - 2 parallel to drive and one perpendicular
class ADIThreeWheelOdometry:public CustomOdometry{
    private:
    void updatePos(); // updates position based on encoder values
    double lVal, mVal, rVal; // readings from the left, middle and right encoders
    double lPrev, mPrev, rPrev; // previous reading from the left, middle and right encoders
    double lDist, rDist, mDist; // offset from the center for the 3 tracking wheels. Used in calculations
    ADIEncoder left, mid, right; // the 3 encoders

    public:
    ADIThreeWheelOdometry(std::tuple<char, char, bool> l, std::tuple<char, char, bool> m, std::tuple<char, char, bool> r); // constructor
    void withDimensions(std::tuple<double, double, double> dimension); // sets the offset of the 3 tracking wheels

    void resetSensors(); // reset sensors
    double getEncoderLeft(); // gets reading from the left encoder
    double getEncoderRight(); // gets reading from the right encoder
    double getEncoderMid(); // gets reading fom the middle encoder
};

// Odometry using 2 encoder that are perpendicular to each other, as well as an imu to gather angle information
class ADITwoWheelIMUOdometry:public CustomOdometry{
    private:
    void updatePos();  // updates position based on encoder values
    double mVal, sVal, aVal; // readings from the side, middle encoder + imu sensor
    double mPrev, sPrev, aPrev; // previous readings from the side, middle encoder + imu sensor
    double mDist, sDist; // offset from the center for the 2 encoders
    okapi::ADIEncoder mid, side; okapi::IMU imu; // sensors used in the system

    public:
    ADITwoWheelIMUOdometry(std::tuple<char, char, bool> s, std::tuple<char, char, bool> m, int port); // constructor
    void withDimensions(std::tuple<double, double> dimension); // sets the offset of the 2 tracking wheels

    void resetSensors(); // reset sensors
    double getEncoderLeft(); // gets reading from the side encoder
    double getEncoderMid(); // gets reading from the middle encoder
};
}