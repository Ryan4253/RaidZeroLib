#pragma once
#include "main.h"

extern Drive drive;
extern CustomOdometry* tracker;
extern Roller* roller;
extern Intake* intake;

void initSubsystems();

void initThreads();
