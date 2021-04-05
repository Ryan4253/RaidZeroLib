#include "main.h"




void debug(){
    int x = 0;
    Robot::startTask("Odometry", OdomController::updatePos, &x);
    Robot::startTask("Display", Robot::displayPosition, &x);
    drive.moveTo({0, 48}, 1, 200_s);
    pros::delay(100);
}

void rL(){

}

void rR(){

}

void bL(){

}

void bR(){

}

void skill(){

}
