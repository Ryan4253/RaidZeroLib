#include "lib4253/Trajectory/Spline/PurePursuitPath.hpp" 
namespace lib4253{

PurePursuitGains::PurePursuitGains(QSpeed iMaxVelocity, QAcceleration iMaxAcceleration, QAngularSpeed iMaxAngularVelocity){
    maxVelocity = iMaxVelocity; maxAcceleration = iMaxAcceleration; maxAngularVelocity = iMaxAngularVelocity;
}

PurePursuitPath::PurePursuitPath(const DiscretePath& iPath, const PurePursuitGains& iLimits): path(iPath){
    velocity.reserve(iPath.size()); acceleration.reserve(iPath.size());;
    velocity.emplace_back(0*mps);
    for(int i = 1; i < path.size()-1; i++){
        velocity.emplace_back(min(iLimits.maxVelocity, iLimits.maxAngularVelocity / iPath.getCurvature(i)));
    }
    velocity.emplace_back(0*mps);

    for(int i = velocity.size()-1; i >= 0; i--){
        QLength dist = iPath[i].distTo(iPath[i+1]);
        velocity[i] = min(velocity[i], sqrt(velocity[i+1]*velocity[i+1] + 2 * iLimits.maxAcceleration * dist));
    }

    for(int i = 0; i < velocity.size()-1; i++){
        QLength dist = iPath[i].distTo(iPath[i+1]);
        velocity[i+1] = min(velocity[i+1], sqrt(velocity[i]*velocity[i] + 2 * iLimits.maxAcceleration * dist));
        acceleration.emplace_back((velocity[i+1] * velocity[i+1] - velocity[i] * velocity[i]) / 2 / dist);
    }
}

int PurePursuitPath::size() const{
    return path.size();
}

Point PurePursuitPath::getPoint(int index) const{
    return path[index];
}

QSpeed PurePursuitPath::getVelocity(int index) const{
    return velocity[index];
}

QAcceleration PurePursuitPath::getAcceleration(int index) const{
    return acceleration[index];
}

QCurvature PurePursuitPath::getCurvature(int index) const{
    return path.getCurvature(index);
}

Point PurePursuitPath::operator[](int index) const{
    return path[index];
}

}
