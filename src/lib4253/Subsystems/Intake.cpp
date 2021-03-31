#include "main.h"

Roller::Roller(Motor a, Motor b):
  top(a.getPort(), a.isReversed(), a.getGearing(), a.getEncoderUnits(), Logger::getDefaultLogger()),
  bottom(b.getPort(), b.isReversed(), b.getGearing(), b.getEncoderUnits(), Logger::getDefaultLogger())
{
}

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

void Roller::execute(){
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
}

void Roller::run(){
  while(true){
    updateState();
    execute();
    pros::delay(3);
  }
}

void Roller::rollerTask(void *ptr){
  pros::delay(10);
  Roller* that = static_cast<Roller*>(ptr);
  that->run();
}

Intake::Intake(Motor a, Motor b):
  left(a.getPort(), a.isReversed(), a.getGearing(), a.getEncoderUnits(), Logger::getDefaultLogger()),
  right(b.getPort(), b.isReversed(), b.getGearing(), b.getEncoderUnits(), Logger::getDefaultLogger())
{
}

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

void Intake::execute(){
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
}

void Intake::run(){
  while(true){
    updateState();
    execute();
    pros::delay(3);
  }
}


void Intake::intakeTask(void *ptr){
  pros::delay(10);
  Intake* that = static_cast<Intake*>(ptr);
  that->run();
}
