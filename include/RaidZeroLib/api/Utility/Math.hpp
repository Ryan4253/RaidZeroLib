#pragma once
#include "RaidZeroLib/api/Units/Units.hpp"
#include <cmath>
#include <memory>
#include <optional>

namespace rz {
using namespace okapi;

template <typename T>
int sgn(T val) {
    return (T(0.0) < val) - (val < T(0.0));
}

QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter);

QSpeed wheelToLinearVelocity(QAngularSpeed velocity, QLength wheelDiameter);

double constrainAngle360(double iAngle);

double constrainAngle180(double iAngle);

QAngle constrainAngle360(QAngle iAngle);

QAngle constrainAngle180(QAngle iAngle);

double sinc(double x);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

std::pair<QSpeed, QSpeed> wheelForwardKinematics(QSpeed linearVelocity, QCurvature curvature, QLength wheelTrack);

std::pair<QAcceleration, QAcceleration> wheelForwardKinematics(QAcceleration linearAcceleration, QCurvature curvature,
                                                               QLength wheelTrack);

}; // namespace rz
