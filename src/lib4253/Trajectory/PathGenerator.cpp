#include "PathGenerator.hpp"
namespace lib4253{

PurePursuitPath PathGenerator::generate(const DiscretePath& iPath, const PurePursuitLimit& limit){
    std::vector<okapi::QSpeed> velocity;
    std::vector<okapi::QAcceleration> acceleration;

    for(int i = 0; i < iPath.getSize(); i++){
         velocity.push_back(min(limit.maxVelocity, limit.k / iPath.getCurvature(i)));
     }

     velocity[velocity.size()-1] = 0 * okapi::mps;

     for(int i = velocity.size()-2; i >= 0; i--){
         okapi::QLength dist = iPath[i].distanceTo(iPath[i+1]);
         velocity[i] = min(velocity[i], sqrt(velocity[i+1]*velocity[i+1] + 2 * limit.maxAcceleration * dist));
     }

     return PurePursuitPath(iPath, limit, velocity);
}


}