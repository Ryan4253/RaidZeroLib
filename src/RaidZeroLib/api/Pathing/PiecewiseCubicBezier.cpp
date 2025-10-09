#include "RaidZeroLib/api/Pathing/PiecewiseCubicBezier.hpp"

namespace rz {

PiecewiseCubicBezier::PiecewiseCubicBezier(std::initializer_list<CubicBezier::Knot> knots) {
    assert(knots.size() >= 2);
    for (auto it = knots.begin(); it != knots.end() - 1; ++it) {
        curves.emplace_back(*it, *(it + 1));
    }
};

Point PiecewiseCubicBezier::getPoint(double t) const noexcept {
    const auto [seg, u] = getIndex(t);
    return curves[seg].getPoint(u);
}

Point PiecewiseCubicBezier::getVelocity(double t) const noexcept {
    const auto [seg, u] = getIndex(t);
    return curves[seg].getVelocity(u);
}

Point PiecewiseCubicBezier::getAcceleration(double t) const noexcept {
    const auto [seg, u] = getIndex(t);
    return curves[seg].getAcceleration(u);
}

PiecewiseCubicBezier::Index PiecewiseCubicBezier::getIndex(double t) const noexcept {
    if (t == 1) {
        return {.segmentIndex = curves.size() - 1, .fractionalIndex = 1};
    }

    const std::size_t segmentIndex = static_cast<std::size_t>(t * curves.size());
    const double fractionalIndex = t * curves.size() - segmentIndex;
    return {segmentIndex, fractionalIndex};
}

} // namespace rz