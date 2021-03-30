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
      rollerState = In;
    }
    else if(true){
      rollerState = Out;
    }
    else if(true){
      rollerState = Autoindex;
    }
    else if(true){
      rollerState = Eject;
    }
    else{
      rollerState = Off;
    }
  }
}

void Roller::run(){
  switch(rollerState){
    case In:
      Robot::setPower({top, bottom}, 127);
      break;

    case Out:
      Robot::setPower({top, bottom}, -127);
      break;

    case Eject:
      eject();
      break;

    case Autoindex:
      autoindex();
      break;

    case Off:
      Robot::setPower({top, bottom}, 0);
      break;
  }
}

void Roller::rollerTask(void *ptr){
  while(true){
    Roller* that = static_cast<Roller*>(ptr);
    that->updateState();
    that->run();
    pros::delay(10);
  }
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
      intakeState = In;
    }
    else if(true){
      intakeState = Out;
    }
    else if(true){
      intakeState = Autoindex;
    }
    else{
      intakeState = Off;
    }
  }
}

void Intake::run(){
  switch(intakeState){
    case In:
      Robot::setPower({intake.left, intake.right}, 127);
      break;
    case Out:
      Robot::setPower({intake.left, intake.right}, -127);
      break;
    case Autoindex:
      break;
    case Off:
      Robot::setPower({intake.left, intake.right}, 0);
      break;
  }
}


void Intake::intakeTask(void *ptr){
  while(true){
    Intake* that = static_cast<Intake*>(ptr);
    that->updateState();
    that->run();
    pros::delay(10);
  }
}
