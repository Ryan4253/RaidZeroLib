#pragma once
#include "lib4253/Splines/Point2D.hpp"
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Utility/TaskWrapper.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include <tuple>
#include <atomic>
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

class OdomDimension{
    public:
    okapi::QLength wheelDiameter{0 * okapi::inch};
    okapi::QLength lDist{0 * okapi::inch}, mDist{0 * okapi::inch}, rDist{0 * okapi::inch};
    double tpr{360};

    OdomDimension(const okapi::QLength& wheelDiam, const okapi::QLength& leftOffset, const okapi::QLength& midOffset, const okapi::QLength& rightOffset);
    OdomDimension(const okapi::QLength& wheelDiam, const okapi::QLength& offset1, const okapi::QLength& offset2);
    ~OdomDimension() = default;
};

class Odometry: public TaskWrapper{
    protected:
    Pose2D globalPos{0, 0, 0}; // the global position of the robot
    OdomDimension dimension{-1 * okapi::inch, -1 * okapi::inch, -1 * okapi::inch, -1 * okapi::inch};

    public:
    Odometry() = default;
    ~Odometry() = default;
    
    static OdomDimension withDimension(const okapi::QLength& wheelDiam, const okapi::QLength& leftOffset, const okapi::QLength& midOffset, const okapi::QLength& rightOffset);
    static OdomDimension withDimension(const okapi::QLength& wheelDiam, const okapi::QLength& offset1, const okapi::QLength& offset2);
    Pose2D getPos() const; // return current position as a struct
    double getX() const; // return x position as a double, units in inches
    okapi::QLength getQX() const; // return x position as type QLength
    double getY() const; // return y position as a double, in unit inches
    okapi::QLength getQY() const; // return y position in type QLength
    double getAngleDeg() const; // return angle in degrees
    double getAngleRad() const; // return angle in radians

    virtual double getEncoderLeft() const; // gets the left encoder reading
    virtual double getEncoderRight() const; // gets the right encoder reading
    virtual double getEncoderMid() const; // gets the mid encoder reading
    virtual double getEncoderSide() const; // gets the side encoder reading

    void setPos(const Pose2D& newPos); // sets the current position
    void setX(const double& x); // sets the x position of the robot
    void setX(const okapi::QLength& inch); // sets the x position of the robot
    void setY(const double& y); // sets the y position of the robot
    void setY(const okapi::QLength& inch); // sets the y position of the robot
    void setAngleDeg(const double& theta); // sets the angle of the robot in degrees
    void setAngleRad(const double& theta); // sets the angle of the robot in radians

    void displayPosition() const; // outputs the x, y, angle of the robot on the robot screen and the console

    void resetState();
    virtual void resetSensors() = 0;
    void reset();
};

// Odometry using 3 encoders - 2 parallel to drive and one perpendicular
class ThreeWheelOdometry : public Odometry{
    private:
    void loop() override; // updates position based on encoder values
    double lVal{0}, mVal{0}, rVal{0}; // readings from the left, middle and right encoders
    double lPrev{0}, mPrev{0}, rPrev{0}; // previous reading from the left, middle and right encoders
    std::shared_ptr<okapi::ContinuousRotarySensor> left{nullptr}, mid{nullptr}, right{nullptr}; // the 3 encoders

    public:
    ThreeWheelOdometry(const std::shared_ptr<okapi::ADIEncoder>& l, const std::shared_ptr<okapi::ADIEncoder>& m, const std::shared_ptr<okapi::ADIEncoder>& r, const OdomDimension& dim); // constructor
    ThreeWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim);
    ~ThreeWheelOdometry() = default;

    void resetSensors() override; // reset sensors
    double getEncoderLeft() const override; // gets reading from the left encoder
    double getEncoderRight() const override; // gets reading from the right encoder
    double getEncoderMid() const override; // gets reading fom the middle encoder
};

// Odometry using 2 encoder that are perpendicular to each other, as well as an imu to gather angle information
class TwoWheelIMUOdometry:public Odometry{
    private:
    void loop() override;  // updates position based on encoder values
    double mVal{0}, sVal{0}, aVal{0}; // readings from the side, middle encoder + imu sensor
    double mPrev{0}, sPrev{0}, aPrev{0}; // previous readings from the side, middle encoder + imu sensor
    std::shared_ptr<okapi::ContinuousRotarySensor> side{nullptr}, mid{nullptr};
    std::shared_ptr<okapi::IMU> inertial{nullptr};

    public:
    TwoWheelIMUOdometry(const std::shared_ptr<okapi::ADIEncoder>& s, const std::shared_ptr<okapi::ADIEncoder>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim); // constructor
    TwoWheelIMUOdometry(const std::shared_ptr<okapi::RotationSensor>& s, const std::shared_ptr<okapi::RotationSensor>& m, const std::shared_ptr<okapi::IMU>& imu, const OdomDimension& dim); // constructor
    TwoWheelIMUOdometry() = default;

    void resetSensors() override; // reset sensors
    double getEncoderSide() const override; // gets reading from the side encoder
    double getEncoderMid() const override; // gets reading from the middle encoder
};

class TwoWheelOdometry : public Odometry{
    private:
    void loop() override;  // updates position based on encoder values
    double lVal{0}, rVal{0}; // readings from the side, middle encoder + imu sensor
    double lPrev{0}, rPrev{0}; // previous readings from the side, middle encoder + imu sensor
    std::shared_ptr<okapi::ContinuousRotarySensor> left{nullptr}, right{nullptr};

    public:
    TwoWheelOdometry(const std::shared_ptr<okapi::ADIEncoder>& l, const std::shared_ptr<okapi::ADIEncoder>& r, const OdomDimension& dim); // constructor
    TwoWheelOdometry(const std::shared_ptr<okapi::RotationSensor>& l, const std::shared_ptr<okapi::RotationSensor>& r, const OdomDimension& dim); // constructor
    ~TwoWheelOdometry() = default;

    void resetSensors() override; // reset sensors
    double getEncoderLeft() const override; // gets reading from the side encoder
    double getEncoderRight() const override; // gets reading from the middle encoder
};
}
