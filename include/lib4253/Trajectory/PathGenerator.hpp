#pragma once
#include "lib4253/Trajectory/Spline/PurePursuitPath.hpp"
namespace lib4253{

class PathGenerator{
    static PurePursuitPath generate(const DiscretePath& iPath, const PurePursuitLimit& limit);
};

}