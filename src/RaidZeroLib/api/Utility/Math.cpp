#include "RaidZeroLib/api/Utility/Math.hpp"

namespace rz {

// QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter) {
//     return velocity / (wheelDiameter / 2) * radian;
// }

// QSpeed wheelToLinearVelocity(QAngularSpeed velocity, QLength wheelDiameter) {
//     return (wheelDiameter / 2) * velocity / radian;
// }

[[nodiscard]] double constrainAngle360(double angle) noexcept {
    return angle - 360.0 * std::floor(angle * (1.0 / 360.0));
}

[[nodiscard]] double constrainAngle180(double angle) noexcept {
    return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

[[nodiscard]] au::QuantityD<au::Radians> constrainAngle360(au::QuantityD<au::Radians> angle) noexcept {
    return au::degrees(constrainAngle360(angle.in(au::degree)));
}

[[nodiscard]] au::QuantityD<au::Radians> constrainAngle180(au::QuantityD<au::Radians> angle) noexcept {
    return au::degrees(constrainAngle180(angle.in(au::degree)));
}

// double sinc(double x) {
//     if (std::abs(x) < 1e-9) {
//         return 1.0 - 1.0 / 6.0 * x * x;
//     } else {
//         return std::sin(x) / x;
//     }
// }

// std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c) {
//     double discriminant = b * b - 4 * a * c;
//     if (discriminant == 0) {
//         return std::make_pair(-b / (2 * a), -b / (2 * a));
//     } else if (discriminant > 0) {
//         return std::make_pair((-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a));
//     }

//     return std::nullopt;
// }

// std::pair<QSpeed, QSpeed> wheelForwardKinematics(QSpeed linearVelocity, QCurvature curvature, QLength wheelTrack) {
//     const auto left = linearVelocity * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
//     const auto right = linearVelocity * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
//     return {left, right};
// }

// std::pair<QAcceleration, QAcceleration> wheelForwardKinematics(QAcceleration linearAcceleration, QCurvature curvature,
//                                                                QLength wheelTrack) {
//     const auto left = linearAcceleration * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
//     const auto right = linearAcceleration * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
//     return {left, right};
// }

} // namespace rz
