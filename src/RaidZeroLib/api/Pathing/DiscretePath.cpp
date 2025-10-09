#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include <algorithm>

namespace rz {
DiscretePath::DiscretePath(std::initializer_list<Point> waypoints) : path(waypoints) {
    assert(!path.empty() && "DiscretePath cannot be empty");
}

DiscretePath DiscretePath::operator+(const DiscretePath& rhs) const {
    std::vector<Point> result;
    result.reserve(size() + rhs.size());

    result.insert(result.end(), path.begin(), path.end());
    result.insert(result.end(), rhs.path.begin(), rhs.path.end());
    return DiscretePath(result);
}

DiscretePath DiscretePath::operator+(const Point& rhs) const {
    std::vector<Point> result;
    result.reserve(size() + 1);

    result.insert(result.end(), path.begin(), path.end());
    result.push_back(rhs);
    return DiscretePath(result);
}

DiscretePath::const_iterator DiscretePath::begin() const noexcept {
    return path.begin();
}

DiscretePath::const_iterator DiscretePath::end() const noexcept {
    return path.end();
}

DiscretePath::const_reverse_iterator DiscretePath::rbegin() const noexcept {
    return path.rbegin();
}

DiscretePath::const_reverse_iterator DiscretePath::rend() const noexcept {
    return path.rend();
}

const Point& DiscretePath::operator[](std::size_t index) const noexcept {
    return path[index];
}

const Point& DiscretePath::front() const noexcept {
    return path.front();
}

const Point& DiscretePath::back() const noexcept {
    return path.back();
}

std::size_t DiscretePath::size() const noexcept {
    return path.size();
}

au::QuantityD<au::Inverse<au::Meters>>  DiscretePath::getCurvature(std::size_t index) const noexcept {
    if (index == 0 || index >= path.size() - 1) {
        return au::ZERO;
    }

    const auto radius = circumradius(path[index - 1], path[index], path[index + 1]);

    if (radius == au::ZERO) {
        return au::ZERO;
    }

    return 1.0 / radius;
}

DiscretePath::const_iterator closestPoint(DiscretePath::const_iterator begin,
                                          DiscretePath::const_iterator end,
                                          const Point& point) {
    const auto proj = [&point](const Point& p) { return p.distTo(point); };
    return std::ranges::min_element(begin, end, std::less<>{}, proj);
}

} // namespace rz