#pragma once
#include "main.h"

/*
* Robot.hpp
*
* This file contains the declaration for common / general functions across
* the robot, such as setting a motor's brakemode or power. It also contains
* the tasks that are currently running on the robot, as well as some misc.
* enums to make programming easier
*/

extern Controller master;

// sensors declarations
extern ADIButton leftAutonSelector;
extern ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;

// possible braketypes for the motors
enum brakeType{
  COAST, BRAKE, HOLD
};

// current state of the match
enum competition{
  OPCONTROL, AUTONOMOUS, INITIALIZE, DISABLED
};

// general functions used for the robot
class Robot{
  private:
    static std::map<std::string, std::unique_ptr<pros::Task>> tasks; // tasks currently running on the robot
    static std::map<std::string, Trajectory> paths;

  public:
    // sets the power of the motors inputted
    static void setPower(MotorGroup motor, double power);
    static void setPower(Motor motor, double power);

    // sets the brakemode of the motor inputted
    static void setBrakeMode(MotorGroup Motor, brakeType mode);
    static void setBrakeMode(Motor motor, brakeType mode);

    // runs the inputted function as a task, set as parameter name
    static void startTask(std::string name, void (*func)(void *), void *param);

    // check if task named "name" is currently running
    static bool taskExists(std::string name);

    // ends the task named "name"
    static void endTask(std::string name);

    static void addPath(std::string name, Trajectory path);

    static Trajectory getPath(std::string name);

    static void deletePath(std::string name);

};

// state of the match
extern competition matchState;
