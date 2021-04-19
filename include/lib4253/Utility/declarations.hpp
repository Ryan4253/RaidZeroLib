#pragma once
#include "main.h"
#include "lib4253/Subsystems/Drive.hpp"
#include "lib4253/Subsystems/Robot.hpp"
#include "lib4253/Subsystems/Intake.hpp"

/*
* declarations.hpp
*
* This file contains declarations for subsystem classes used on
* the robot. It is written to ensure easy declaration for the user
* as each robot uses different subsystems, and functions in this
* file are intended to be changed based on different teams
*/

// subsystems on the robot
extern Drive drive;
extern CustomOdometry* tracker;
extern Roller* roller;
extern Intake* intake;

// initializes individual subsystems
void initSubsystems();

// starts the task for each subsystem
void initThreads();
