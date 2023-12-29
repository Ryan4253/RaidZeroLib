#include "RaidZeroLib/api/Pathing/PiecewiseCubicBezier.hpp"

namespace rz {

PiecewiseCubicBezier::PiecewiseCubicBezier(std::initializer_list<CubicBezier::Knot> knots) {
    for (auto it = knots.begin(); it != knots.end() - 1; ++it) {
        curves.emplace_back(CubicBezier(*it, *(it + 1)));
    }
};

Point PiecewiseCubicBezier::getPoint(double t) const {
    const auto [index, tIndex] = getIndex(t);
    return curves[index].getPoint(tIndex);
}

Point PiecewiseCubicBezier::getVelocity(double t) const {
    const auto [index, tIndex] = getIndex(t);
    return curves[index].getVelocity(tIndex);
}

Point PiecewiseCubicBezier::getAcceleration(double t) const {
    const auto [index, tIndex] = getIndex(t);
    return curves[index].getAcceleration(tIndex);
}

std::pair<int, double> PiecewiseCubicBezier::getIndex(double t) const {
    if (t == 1) {
        return std::make_pair(curves.size() - 1, 1);
    }

    const int index = (int)(t * curves.size());
    const double tIndex = t * curves.size() - index;
    return std::make_pair(index, tIndex);
}

} // namespace rz