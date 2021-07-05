#include "main.h"
#include "declarations.hpp"


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
            //auton--;
        }
        else if(rState && !rPreValue){
            //auton++;
        }
        else{
            time = 0;
            pros::lcd::clear_line(5);
        }

        lPreValue = lState;
        rPreValue = rState;

        //auton = (auton + 4) % 4;

        pros::lcd::print(3, "Current Autonomous: %d", 1/*auton*/);

        if(time > 1000){
            break;
        }

        pros::delay(75);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); pros::lcd::print(1, "INITIALIZE");
    //matchState = INITIALIZE;

    initSubsystems();
    //initThreads();
    initPaths();

    //autonSelector();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize(){}