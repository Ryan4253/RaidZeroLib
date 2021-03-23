#include "main.h"

typedef void(*stuff)();

void debug(){
    int x = 0;
    Odom({0, 0, 90});
    Robot::startTask("Odometry", Odom::updatePos, &x);
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

stuff run[6] = {debug, rL, rR, bL, bR, skill};
