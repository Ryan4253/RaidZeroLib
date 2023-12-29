#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"

namespace rz {

QLength ParametricPath::getX(double t) const {
    return getPoint(t).X();
}

QLength ParametricPath::getY(double t) const {
    return getPoint(t).Y();
}

QLength ParametricPath::getdX(double t) const {
    return getVelocity(t).X();
}

QLength ParametricPath::getdY(double t) const {
    return getVelocity(t).Y();
}

QLength ParametricPath::getddX(double t) const {
    return getAcceleration(t).X();
}

QLength ParametricPath::getddY(double t) const {
    return getAcceleration(t).Y();
}

Rotation ParametricPath::getTheta(double t) const {
    return Rotation(getdX(t), getdY(t));
}

QCurvature ParametricPath::getCurvature(double t) const {
    const Point velocity = getVelocity(t);
    const Point acceleration = getAcceleration(t);

    if (velocity.mag().getValue() == 0) {
        return 0 * radpm;
    }

    return radian * velocity.wedge(acceleration) / cube(velocity.mag());
}

QLength ParametricPath::getLength(double tStart, double tEnd) const {
    const double dT = 0.01;

    QLength length{0.0};
    for (double t = tStart; t < tEnd; t += dT) {
        length += getPoint(t).distTo(getPoint(t + dT));
    }

    return length;
}

DiscretePath ParametricPath::toDiscrete(int numSegments, bool end) const {
    std::vector<Point> path;
    const double increment = 1.0 / (numSegments);

    for (int i = 0; i < numSegments; i++) {
        path.emplace_back(getPoint(i * increment));
    }

    if (end) {
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

DiscretePath ParametricPath::toDiscrete(QLength distance, bool end) const {
    const double dT = 0.001;

    QLength traversed{0.0};
    std::vector<Point> path;
    path.emplace_back(getPoint(0));

    for (double t = 0; t < 1; t += dT) {
        traversed += getPoint(t).distTo(getPoint(t + dT));
        if (traversed >= distance) {
            traversed = 0_m;
            path.emplace_back(getPoint(t));
        }
    }

    if (path.back().distTo(getPoint(1)) < distance / 2) {
        path.pop_back();
    }

    if (end) {
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

double ParametricPath::stepT(double t, QLength distance) const {
    return t + (distance / getVelocity(t).mag()).convert(number);
}

} // namespace rz