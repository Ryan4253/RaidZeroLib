#include "lib4253/Trajectory/Spline/PurePursuitPath.hpp" 
namespace lib4253{

PurePursuitPath::PurePursuitPath(const DiscretePath& iPath, const PurePursuitLimit& limit, std::vector<okapi::QSpeed> iVelocity){
    path = iPath;
    velocity = iVelocity;
}

int PurePursuitPath::getSize(int index){
    return path.getSize();
}

Point2D PurePursuitPath::getPoint(int index){
    return path[index];
}

okapi::QSpeed PurePursuitPath::getVelocity(int index){
    return velocity[index];
}

okapi::QCurvature PurePursuitPath::getCurvature(int index){
    return path.getCurvature(index);
}

}
