#pragma once
#include "RaidZeroLib/api/Units/Units.hpp"
#include <cmath>
#include <memory>
#include <optional>

namespace rz {

template <typename T>
int sgn(T val) {
    return (T(0.0) < val) - (val < T(0.0));
}

// QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter);

// QSpeed wheelToLinearVelocity(QAngularSpeed velocity, QLength wheelDiameter);

[[nodiscard]] double constrainAngle360(double angle) noexcept;

[[nodiscard]] double constrainAngle180(double angle) noexcept;

[[nodiscard]] au::QuantityD<au::Radians> constrainAngle360(au::QuantityD<au::Radians> angle) noexcept;

[[nodiscard]] au::QuantityD<au::Radians> constrainAngle180(au::QuantityD<au::Radians> angle) noexcept;



// double sinc(double x);

// std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

// std::pair<QSpeed, QSpeed> wheelForwardKinematics(QSpeed linearVelocity, QCurvature curvature, QLength wheelTrack);

// std::pair<QAcceleration, QAcceleration> wheelForwardKinematics(QAcceleration linearAcceleration, QCurvature curvature,
//                                                                QLength wheelTrack);

}; // namespace rz
