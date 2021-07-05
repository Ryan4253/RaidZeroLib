#pragma once
#include "main.h"
/*
* declarations.hpp
*
* This file contains declarations for subsystem classes used on
* the robot. It is written to ensure easy declaration for the user
* as each robot uses different subsystems, and functions in this
* file are intended to be changed based on different teams
*/
extern okapi::Controller master;

extern okapi::ADIButton leftAutonSelector;
extern okapi::ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;

extern std::shared_ptr<lib4253::Motor> leftBack;
extern std::shared_ptr<lib4253::Motor> leftFront;
extern std::shared_ptr<lib4253::Motor> leftTop;
extern std::shared_ptr<lib4253::Motor> rightBack;
extern std::shared_ptr<lib4253::Motor> rightFront;
extern std::shared_ptr<lib4253::Motor> rightTop;

extern std::shared_ptr<lib4253::Odometry> odom;
extern std::shared_ptr<lib4253::Chassis> chassis;
extern std::shared_ptr<lib4253::OdomController> odomController;

// initializes individual subsystems
void initSubsystems();

// starts the task for each subsystem
void initThreads();

void initPaths();


