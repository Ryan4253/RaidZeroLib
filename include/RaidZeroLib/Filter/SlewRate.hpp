#pragma once
#include <algorithm>
#include "okapi/api/filter/filter.hpp"

namespace rz{
using namespace okapi;

class SlewRate : public Filter{
    public:    

    SlewRate(double iAccStep, double iDecStep);

    SlewRate(double iStep);

    ~SlewRate() = default;

    double filter(double iInput) override;

    double getOutput() const override;

    protected:
    double speed{0.0};
    double accStep, decStep;
};
}