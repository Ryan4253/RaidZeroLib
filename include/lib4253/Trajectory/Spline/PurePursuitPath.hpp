#pragma once
#include "lib4253/Trajectory/Spline/DiscretePath.hpp"
#include "lib4253/Utility/Units.hpp"
namespace lib4253{

class PathGenerator;

struct PurePursuitLimit{
    okapi::QSpeed maxVelocity;
    okapi::QAcceleration maxAcceleration;
    okapi::QAngularSpeed k;
};

class PurePursuitPath{
    public:
    int getSize(int index);
    Point2D getPoint(int index);
    okapi::QSpeed getVelocity(int index);
    okapi::QCurvature getCurvature(int index);

    protected:
    // dont interface with this class directly, let pathgenerator do the work
    PurePursuitPath(const DiscretePath& iPath, const PurePursuitLimit& iLimit, std::vector<okapi::QSpeed> iVelocity);
    friend class PathGenerator;

    private:
    DiscretePath path;
    std::vector<okapi::QSpeed> velocity;
    PurePursuitLimit limit;

};

}