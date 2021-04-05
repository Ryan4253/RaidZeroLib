#include "main.h"

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
  pros::lcd::initialize();
  pros::lcd::print(1, "INITIALIZE");
  Robot::setBrakeMode(base, COAST);
  drive.resetEncoders(); drive.resetIMU();

  OdomController('A', 'B', 'C', 'D', 'E', 'F');
  drive.drivePID.withGain(1, 0, 0).withIGain(500, 12).withEMAGain(0.35).initialize();
  drive.turnPID.withGain(1, 0, 0).withIGain(500, 12).withEMAGain(0.35).initialize();
  drive.PPTenshi.withGain(0, 0, 0).withMaxVel(1).withMaxAccel(1).withTurnGain(2).initialize();
  drive.driveSlew.withStep(256, 9);

  autonSelector();
}

void competition_initialize(){}

void disabled() {}
