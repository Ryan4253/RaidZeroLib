#include "main.h"
#include "lib4253/Utility/declarations.hpp"

void debug(){
    int x = 0;
    Robot::startTask("ODOM", CustomOdometry::odomTask, tracker);
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
