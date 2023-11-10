#pragma once
#include "RaidZeroLib/api/Units/Units.hpp"
#include <cmath>
#include <memory>
#include <optional>

namespace rz {
using namespace okapi;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter);

double constrainAngle360(double iAngle);

double constrainAngle180(double iAngle);

QAngle constrainAngle360(QAngle iAngle);

QAngle constrainAngle180(QAngle iAngle);

double sinc(double x);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

template <typename T> std::pair<T, T> wheelForwardKinematics(T kinematics, QCurvature curvature, QLength wheelTrack) {
    const T left = kinematics * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    const T right = kinematics * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
    return {left, right};
}

}; // namespace rz
