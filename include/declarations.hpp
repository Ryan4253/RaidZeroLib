#pragma once
#include "main.h"
#include "lib4253/Subsystems/Drive.hpp"
#include "lib4253/Subsystems/Robot.hpp"
#include "lib4253/Subsystems/Intake.hpp"

extern Drive drive;
extern CustomOdometry* tracker;
extern Roller* roller;
extern Intake* intake;

void initSubsystems();

void initThreads();
