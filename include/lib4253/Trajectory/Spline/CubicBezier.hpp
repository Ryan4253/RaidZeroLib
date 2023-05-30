#pragma once
#include "lib4253/Trajectory/Spline/Spline.hpp"
#include "lib4253/Utility/Math.hpp"
#include <stack>
#include<vector>
namespace lib4253{
using namespace okapi;

class CubicBezier : public Spline{
    public:
    struct Knot{
		QLength x, y; QAngle angle; QLength mag = 1 * meter;
		Knot(QLength ix, QLength iy, QAngle iangle, QLength imag = 1 * meter);
	};

    CubicBezier(const Point& iStart, const Point& iControl1, const Point& iControl2, const Point& iEnd);

    CubicBezier(const Knot& iStart, const Knot& iEnd);

    Point getPoint(double t) const override;;

    DiscretePath generate(int iStep, bool iEnd) const override;

    QLength getLength() const override;

    Point getVelocity(double t) const;

    Point getAcceleration(double t) const;

    QCurvature getCurvature(double t) const;

    private:
    Point start, control1, control2, end;
};
}
