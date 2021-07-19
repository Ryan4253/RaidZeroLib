#include "main.h"
#include "declarations.hpp"


void debug(){
    int x = 0;
    //Robot::startTask("ODOM", CustomOdometry::odomTask, tracker);
    //drive.moveTo({0, 48}, 1, 200*okapi::second);
    pros::delay(100);

    using namespace lib4253;
    
}

void test(){
    using namespace okapi;
    lib4253::TrapezoidalMotionProfile<QLength> lmp(0_mps, 0_mps2, 0_mps3);
    lmp.setDistance(5_m);
    std::pair<QSpeed, QAcceleration> result = lmp.calculate(3_m);
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
