#pragma once
#include "lib4253/Trajectory/Geometry/Point.hpp"
#include "lib4253/Trajectory/Spline/DiscretePath.hpp"

namespace lib4253{
using namespace okapi;

class Spline{
    public:
	virtual DiscretePath generate(int iStep, bool iEnd = true) const = 0;
	virtual Point getPoint(double t) const = 0;
    virtual QLength getLength() const = 0;
};


}