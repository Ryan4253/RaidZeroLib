#include "main.h"

class Ramsete{
    double beta, omega;
    std::vector<TrajectoryPoint> waypoint;
    
    Ramsete(const double& iBeta, const double& iOmega);
    
    void setGain(const double& iBeta, const double& iOmega);
    
    void update();

};
