#include "main.h"
#include "lib4253/Utility/declarations.hpp"
using namespace lib4253;


void autonSelector(){
  int lPreValue = 0;
  int rPreValue = 0;
  int time = 0;

  while(true){
    bool lState = leftAutonSelector.isPressed();
    bool rState = rightAutonSelector.isPressed();

    if(lState && rState){
     time += 75;
     pros::lcd::print(5, "Escape Progress: %d", 1000 - time);
    }
    else if(lState && !lPreValue){
      auton--;
    }
    else if(rState && !rPreValue){
      auton++;
    }
    else{
      time = 0;
      pros::lcd::clear_line(5);
    }

    lPreValue = lState;
    rPreValue = rState;

    auton = (auton + 4) % 4;

    pros::lcd::print(3, "Current Autonomous: %d", auton);

    if(time > 1000){
      break;
    }

    pros::delay(75);
  }
}

void initialize() {
  pros::lcd::initialize(); pros::lcd::print(1, "INITIALIZE");
  matchState = INITIALIZE;

  initSubsystems();
  //initThreads();
  initPaths();

  //autonSelector();
}

void competition_initialize(){}

void disabled() {}
