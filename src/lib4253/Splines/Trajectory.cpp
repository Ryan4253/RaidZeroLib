#include "main.h"

TrajectoryPoint::TrajectoryPoint(double v, double a){
    velocity = v, acceleration = a;
}

Trajectory::Trajectory(std::vector<TrajectoryPoint> l, std::vector<TrajectoryPoint> r){
    left = l;
    right = r;
}

int Trajectory::getSize(){
    return left.size();
}

std::pair<TrajectoryPoint, TrajectoryPoint> Trajectory::getKinematics(int index){
    if(index >= left.size()){
        TrajectoryPoint left = {0, 0};
        TrajectoryPoint right = {0, 0};
        return std::make_pair(left, right);
    }
    else{
        return std::make_pair(left[index], right[index]);
    }
}
