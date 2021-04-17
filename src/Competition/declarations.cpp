#include "main.h"
#include "declarations.hpp"

CustomOdometry* tracker = new ADIThreeWheelOdometry({'A', 'B', true}, {'C', 'D', false}, {'E', 'F', false});
Drive drive({-10, 9}, {8, -7});
//Roller* roller = new Roller();
//Intake* intake = new Intake();

void initSubsystems(){
  drive
    .withOdometry(tracker)
    .withDrivePID({0, 0, 0}, {1, 1}, {1})
    .withTurnPID({0, 0, 0}, {1, 1}, {1})
    .withPurePursuit({0, 0, 0}, {2}, {1, 1})
    .withSlew(256, 9)
    .initialize();

  tracker->withDimensions({3.389, 5.748, 3.389});
  tracker->reset();
}

void initThreads(){
  Robot::startTask("Drive", Drive::driveTask, &drive);
  Robot::startTask("Odom", CustomOdometry::odomTask, tracker);
}
