#pragma once
#include "RaidZeroLib/Geometry/Point.hpp"
#include "RaidZeroLib/Utility/Units.hpp"
#include <cmath>
#include <memory>
#include <optional>

namespace rz{
using namespace okapi;
class Translation;

double constrainAngle360(double iAngle);

double constrainAngle180(double iAngle);

QAngle constrainAngle360(QAngle iAngle);

QAngle constrainAngle180(QAngle iAngle);

double sinc(double x);

QLength circumradius(const Translation& iLeft, const Translation& iMid, const Translation& iRight);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

};
