#pragma once
#include "RaidZeroLib/Units/Units.hpp"
#include <cmath>
#include <memory>
#include <optional>

namespace rz{
using namespace okapi;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double constrainAngle360(double iAngle);

double constrainAngle180(double iAngle);

QAngle constrainAngle360(QAngle iAngle);

QAngle constrainAngle180(QAngle iAngle);

double sinc(double x);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

std::pair<QSpeed, QSpeed> curvatureToWheelVelocity(QSpeed speed, QCurvature curvature, QLength wheelTrack, bool isReversed = false);

std::pair<QAcceleration, QAcceleration> curvatureToWheelAcceleration(QAcceleration accel, QCurvature curvature, QLength wheelTrack, bool isReversed = false);

};
