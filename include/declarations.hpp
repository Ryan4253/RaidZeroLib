#pragma once
#include "main.h"
#include "lib4253/Subsystems/Drive.hpp"
#include "lib4253/Subsystems/Robot.hpp"
/*
* declarations.hpp
*
* This file contains declarations for subsystem classes used on
* the robot. It is written to ensure easy declaration for the user
* as each robot uses different subsystems, and functions in this
* file are intended to be changed based on different teams
*/
extern okapi::Controller master;

// sensors declarations
extern okapi::ADIButton leftAutonSelector;
extern okapi::ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;
// subsystems on the robot
extern lib4253::Drive drive;
extern lib4253::CustomOdometry* tracker;

// initializes individual subsystems
void initSubsystems();

// starts the task for each subsystem
void initThreads();

void initPaths();
