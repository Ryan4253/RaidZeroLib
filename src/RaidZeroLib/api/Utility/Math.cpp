#include "RaidZeroLib/api/Utility/Math.hpp"

namespace rz {

QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter) {
    return radian * velocity / (wheelDiameter / 2);
}

double constrainAngle360(double iAngle) {
    return iAngle - 360.0 * std::floor(iAngle * (1.0 / 360.0));
}

double constrainAngle180(double iAngle) {
    return iAngle - 360.0 * std::floor((iAngle + 180.0) * (1.0 / 360.0));
}

QAngle constrainAngle360(QAngle iAngle) {
    return constrainAngle360(iAngle.convert(degree)) * degree;
}

QAngle constrainAngle180(QAngle iAngle) {
    return constrainAngle180(iAngle.convert(degree)) * degree;
}

double sinc(double x) {
    if (std::abs(x) < 1e-9) {
        return 1.0 - 1.0 / 6.0 * x * x;
    } else {
        return std::sin(x) / x;
    }
}

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c) {
    double discriminant = b * b - 4 * a * c;
    if (discriminant == 0) {
        return std::make_pair(-b / (2 * a), -b / (2 * a));
    } else if (discriminant > 0) {
        return std::make_pair((-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a));
    }

    return std::nullopt;
}

} // namespace rz
