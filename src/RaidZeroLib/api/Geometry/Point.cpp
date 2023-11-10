#include "RaidZeroLib/api/Geometry/Point.hpp"
namespace rz {

Translation::Translation(QLength iX, QLength iY) : x(iX), y(iY) {
}

Translation::Translation(QLength iMag, const Rotation& iAngle) : x(iMag * iAngle.Cos()), y(iMag * iAngle.Sin()) {
}

Translation::Translation(const Translation& rhs) : x(rhs.x), y(rhs.y) {
}

QLength Translation::X() const {
    return x;
}

QLength Translation::Y() const {
    return y;
}

void Translation::setX(QLength iX) {
    x = iX;
}

void Translation::setY(QLength iY) {
    y = iY;
}

Translation Translation::operator+(const Translation& rhs) const {
    return Point(x + rhs.x, x + rhs.y);
}

Translation Translation::operator-(const Translation& rhs) const {
    return -rhs + *this;
}

Translation Translation::operator-() const {
    return *this * -1;
}

Translation Translation::operator*(double scalar) const {
    return Point(x * scalar, y * scalar);
}

Translation Translation::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

bool Translation::operator==(const Translation& rhs) const {
    return abs(x - rhs.x) < 1E-9 * meter && abs(y - rhs.y) < 1E-9 * meter;
}

bool Translation::operator!=(const Translation& rhs) const {
    return !(*this == rhs);
}

void Translation::operator=(const Translation& rhs) {
    x = rhs.x, y = rhs.y;
}

QAngle Translation::theta() const {
    return atan2(y, x);
}

QLength Translation::mag() const {
    return hypot(x, y);
}

QLength Translation::distTo(const Translation& rhs) const {
    return hypot(rhs.x - x, rhs.y - y);
}

QAngle Translation::angleTo(const Translation& rhs) const {
    return theta() - rhs.theta();
}

QArea Translation::dot(const Translation& rhs) const {
    return x * rhs.x + y * rhs.y;
}

QArea Translation::wedge(const Translation& rhs) const {
    return x * rhs.y - y * rhs.x;
}

Translation Translation::project(const Translation& rhs) const {
    return rhs * (this->dot(rhs) / this->dot(*this)).convert(number);
}

Translation Translation::rotateBy(const Rotation& rhs) const {
    return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

QLength circumradius(const Translation& iLeft, const Translation& iMid, const Translation& iRight) {
    Point A = iLeft;
    Point B = iMid;
    Point C = iRight;

    QLength a = B.distTo(C);
    QLength b = A.distTo(C);
    QLength c = A.distTo(B);
    auto a2 = a * a, b2 = b * b, c2 = c * c;

    Point pa = A * (a2 * (b2 + c2 - a2) / ((b + c) * (b + c) - a2) / (a2 - (b - c) * (b - c))).convert(number);
    Point pb = B * (b2 * (a2 + c2 - b2) / ((a + c) * (a + c) - b2) / (b2 - (a - c) * (a - c))).convert(number);
    Point pc = C * (c2 * (a2 + b2 - c2) / ((a + b) * (a + b) - c2) / (c2 - (a - b) * (a - b))).convert(number);

    Point center = pa + pb + pc;

    QLength radius = center.distTo(A);

    return radius;
}

std::optional<double> circleLineIntersection(const Translation& start, const Translation& end, const Translation& point,
                                             QLength radius) {
    const Point d = end - start;
    const Point f = start - point;

    const auto a = d.dot(d);
    const auto b = 2 * (f.dot(d));
    const auto c = f.dot(f) - radius * radius;
    const auto discriminant = b * b - 4 * a * c;

    if (discriminant.getValue() >= 0) {
        const auto dis = sqrt(discriminant);
        const double t1 = ((-1 * b - dis) / (2 * a)).convert(number);
        const double t2 = ((-1 * b + dis) / (2 * a)).convert(number);

        if (t2 >= 0 && t2 <= 1) {
            return t2;
        } else if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
    }

    return std::nullopt;
}

} // namespace rz