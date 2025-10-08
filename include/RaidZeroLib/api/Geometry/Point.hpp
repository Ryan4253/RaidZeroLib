#pragma once
#include "au/au.hpp"
#include <optional>

namespace rz {

class Rotation;

class Point {
    public:
    constexpr Point() noexcept = default;

    Point(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept;

    Point(au::QuantityD<au::Meters> magnitude, const Rotation& rotation) noexcept;

    au::QuantityD<au::Meters> X() const noexcept;

    au::QuantityD<au::Meters> Y() const noexcept;

    Point operator+(const Point& rhs) const noexcept;

    Point operator-(const Point& rhs) const noexcept;

    Point operator-() const noexcept;

    Point operator*(double scalar) const noexcept;

    Point operator/(double scalar) const noexcept;

    au::QuantityD<au::Radians> theta() const noexcept;

    au::QuantityD<au::Meters> mag() const noexcept;

    au::QuantityD<au::Meters> distTo(const Point& rhs) const noexcept;

    au::QuantityD<au::Radians> angleTo(const Point& rhs) const noexcept;

    au::QuantityD<au::Squared<au::Meters>> dot(const Point& rhs) const noexcept;

    au::QuantityD<au::Squared<au::Meters>> wedge(const Point& rhs) const noexcept;

    Point project(const Point& rhs) const noexcept;

    Point rotateBy(const Rotation& rhs) const noexcept;

    bool isApprox(const Point& rhs) const noexcept;

    private:
    au::QuantityD<au::Meters> x = au::ZERO;
    au::QuantityD<au::Meters> y = au::ZERO;
};

au::QuantityD<au::Meters> circumradius(const Point& A, const Point& B, const Point& C) noexcept;

std::optional<double> circleLineIntersection(const Point& start, const Point& end, const Point& center,
                                             au::QuantityD<au::Meters> radius) noexcept;

} // namespace rz
