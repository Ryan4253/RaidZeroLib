#include "main.h"

void Intakes::Roller::setState(rollerState i){
  state = i;
}

rollerState Intakes::Roller::getState(){
  return state;
}

void Intakes::Roller::eject(){
  Robot::setPower({Roller::top}, -127);
  Robot::setPower({Roller::bottom}, 127);
}

intakeState Intakes::Intake::getState(){
  return state;
}

void Intakes::Intake::setState(intakeState i){
  state = i;
}

void Intakes::intakeTask(void *ptr){
  if(matchState == OPCONTROL){
    if(true){
      intake.setState(iIn);
    }
    else if(true){
      intake.setState(iOut);
    }
    else if(true){
      intake.setState(iAutoIndex);
    }
  }

  switch(intake.getState()){
    case iIn:
      Robot::setPower({intake.left, intake.right}, 127);
      break;
    case iOut:
      Robot::setPower({intake.left, intake.right}, -127);
      break;
    case iAutoIndex:
      intakeSystem.autoIndex();
      break;
    case iOff:
      Robot::setPower({intake.left, intake.right}, 0);
      break;
  }
}
