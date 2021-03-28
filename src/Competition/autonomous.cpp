#include "main.h"
//#include "auton.cpp"

void autonomous() {
  //run[0]();
  intakeSystem.intake.setState(Intakes::Intake::intakeState::in);
}
