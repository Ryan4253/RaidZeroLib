#pragma once
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api.hpp"

namespace lib4253{
using namespace okapi;

class ExpandedSkidSteerModel : public okapi::SkidSteerModel{
    public:
    void curvature(double iThrottle, double iCurvature, double iThreshold = 0);

};

}