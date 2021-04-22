#include "main.h"
#include "lib4253/Utility/declarations.hpp"

CustomOdometry* tracker = new ADIThreeWheelOdometry({'A', 'B', true}, {'C', 'D', false}, {'E', 'F', false});
Drive drive({-10, 9}, {8, -7});
//Roller* roller = new Roller();
//Intake* intake = new Intake();

void initSubsystems(){
  tracker->withDimensions({3.389, 5.748, 3.389});
  tracker->reset();

  drive
    .withOdometry(tracker)
    .withDimensions({4.35}, {36, 84}, {12})
    .withDrivePID({0, 0, 0}, {1, 1}, {1})
    .withTurnPID({0, 0, 0}, {1, 1}, {1})
    .withPurePursuit({6}, {2}, {1, 1})
    .withSlew(256, 9)
    .initialize();
}

void initThreads(){
  Robot::startTask("Drive", Drive::driveTask, &drive);
  Robot::startTask("Odom", CustomOdometry::odomTask, tracker);
}
