#include "RaidZeroLib/api/Utility/Math.hpp"

namespace rz {

QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter) {
    return velocity / (wheelDiameter / 2) * radian;
}

QSpeed wheelToLinearVelocity(QAngularSpeed velocity, QLength wheelDiameter) {
    return (wheelDiameter / 2) * velocity / radian;
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

std::pair<QSpeed, QSpeed> wheelForwardKinematics(QSpeed linearVelocity, QCurvature curvature, QLength wheelTrack) {
    const auto left = linearVelocity * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    const auto right = linearVelocity * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    return {left, right};
}

std::pair<QAcceleration, QAcceleration> wheelForwardKinematics(QAcceleration linearAcceleration, QCurvature curvature,
                                                               QLength wheelTrack) {
    const auto left = linearAcceleration * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    const auto right = linearAcceleration * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    return {left, right};
}

} // namespace rz
