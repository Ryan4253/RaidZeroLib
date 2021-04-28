#include "main.h"
#include "Intake.hpp"
#include "Robot.hpp"

namespace lib4253{

Roller::Roller(int tPort, int bPort):
  top(tPort), bottom(bPort)
{}

void Roller::setState(State i){
  if(rollerState != i){
      rollerState = i;
  }
}

Roller::State Roller::getState(){
  return rollerState;
}

void Roller::eject(){
  Robot::setPower({top}, -127);
  Robot::setPower({bottom}, 127);
}

void Roller::autoindex(){

}

void Roller::updateState(){
  if(matchState == OPCONTROL){
    if(true){
      rollerState = IN;
    }
    else if(true){
      rollerState = OUT;
    }
    else if(true){
      rollerState = AUTOINDEX;
    }
    else if(true){
      rollerState = EJECT;
    }
    else{
      rollerState = OFF;
    }
  }
}

void Roller::run(){
  while(true){
    updateState();

    switch(rollerState){
      case IN:
        Robot::setPower({top, bottom}, 127);
        break;

      case OUT:
        Robot::setPower({top, bottom}, -127);
        break;

      case EJECT:
        eject();
        break;

      case AUTOINDEX:
        autoindex();
        break;

      case OFF:
        Robot::setPower({top, bottom}, 0);
        break;
    }

    pros::delay(3);
  }
}

void Roller::rollerTask(void *ptr){
  pros::delay(10);
  Roller* that = static_cast<Roller*>(ptr);
  that->run();
}

Intake::Intake(int lPort, int rPort):
  left(lPort), right(rPort)
{}

Intake::State Intake::getState(){
  return intakeState;
}

void Intake::setState(State i){
  if(intakeState != i){
      intakeState = i;
  }
}

void Intake::updateState(){
  if(matchState == OPCONTROL){
    if(true){
      intakeState = IN;
    }
    else if(true){
      intakeState = OUT;
    }
    else if(true){
      intakeState = AUTOINDEX;
    }
    else{
      intakeState = OFF;
    }
  }
}

void Intake::run(){
  while(true){
    updateState();

    switch(intakeState){
      case IN:
        Robot::setPower({left, right}, 127);
        break;
      case OUT:
        Robot::setPower({left, right}, -127);
        break;
      case AUTOINDEX:
        break;
      case OFF:
        Robot::setPower(left, 0);
        break;
    }

    pros::delay(3);
  }
}


void Intake::intakeTask(void *ptr){
  pros::delay(10);
  Intake* that = static_cast<Intake*>(ptr);
  that->run();
}

}
