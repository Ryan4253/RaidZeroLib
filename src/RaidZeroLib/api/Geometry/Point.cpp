#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"

namespace rz {

Point::Point(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept : x(x), y(y) {
}

Point::Point(au::QuantityD<au::Meters> magnitude, const Rotation& angle) noexcept
    : x(magnitude * angle.Cos()), y(magnitude * angle.Sin()) {}

au::QuantityD<au::Meters> Point::X() const noexcept {
    return x;
}

au::QuantityD<au::Meters> Point::Y() const noexcept {
    return y;
}

Point Point::operator+(const Point& rhs) const noexcept {
    return Point(x + rhs.x, y + rhs.y);
}

Point Point::operator-(const Point& rhs) const noexcept {
    return -rhs + *this;
}

Point Point::operator-() const noexcept {
    return *this * -1;
}

Point Point::operator*(double scalar) const noexcept {
    return Point(x * scalar, y * scalar);
}

Point Point::operator/(double scalar) const noexcept {
    return *this * (1.0 / scalar);
}

au::QuantityD<au::Radians> Point::theta() const noexcept {
    if(x == au::ZERO && y == au::ZERO){
        return au::ZERO;
    }

    return au::arctan2(y, x);
}

au::QuantityD<au::Meters> Point::mag() const noexcept {
    return au::hypot(x, y);
}

au::QuantityD<au::Meters> Point::distTo(const Point& rhs) const noexcept {
    return au::hypot(rhs.x - x, rhs.y - y);
}

au::QuantityD<au::Radians> Point::angleTo(const Point& rhs) const noexcept {
    return constrainAngle180(rhs.theta() - theta());
}

au::QuantityD<au::Squared<au::Meters>> Point::dot(const Point& rhs) const noexcept {
    return x * rhs.x + y * rhs.y;
}

au::QuantityD<au::Squared<au::Meters>> Point::wedge(const Point& rhs) const noexcept {
    return x * rhs.y - y * rhs.x;
}

Point Point::project(const Point& rhs) const noexcept {
    return rhs * (this->dot(rhs) / rhs.dot(rhs));
}

Point Point::rotateBy(const Rotation& rhs) const noexcept {
    const double c = rhs.Cos();
    const double s = rhs.Sin();
    return Point(x * c - y * s, x * s + y * c);
}

bool Point::isApprox(const Point& rhs) const noexcept {
    return this->distTo(rhs) <= au::meters(1e-9);
}

au::QuantityD<au::Meters> circumradius(const Point& A, const Point& B, const Point& C) noexcept {
    // Circumradius of a triangle with vertex A, B, C is (a * b * c) / (4 * area)
    // https://artofproblemsolving.com/wiki/index.php/Circumradius

    const auto a = B.distTo(C);
    const auto b = A.distTo(C);
    const auto c = A.distTo(B); 

    const auto area = au::abs((B - A).wedge(C - A)) / 2;

    if (area == au::ZERO) {
        return au::meters(0.0);
    }

    return (a * b * c) / (area * 4);
}

std::optional<double> circleLineIntersection(const Point& start, const Point& end, const Point& center,
                                             au::QuantityD<au::Meters> radius) noexcept {
    const Point d = end - start;
    const Point f = start - center;

    const auto a = d.dot(d);
    const auto b = 2 * (f.dot(d));
    const auto c = f.dot(f) - radius * radius;
    const auto discriminant = b * b - 4 * a * c;

    if (discriminant >= au::ZERO) {
        const auto dis = sqrt(discriminant);
        const double t1 = ((-1 * b - dis) / (2 * a));
        const double t2 = ((-1 * b + dis) / (2 * a));

        if (t2 >= 0 && t2 <= 1) {
            return t2;
        } else if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
    }

    return std::nullopt;
}

} // namespace rz