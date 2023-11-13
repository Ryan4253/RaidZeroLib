#pragma once
#include "RaidZeroLib/api/Utility/Util.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include <concepts>

namespace rz {
using namespace okapi;

template <isRQuantity Distance>
class FeedforwardController {
    public:
    using Velocity = decltype(Distance{1.0} / QTime{1.0});
    using Acceleration = decltype(Velocity{1.0} / QTime{1.0});

    virtual double calculate(Velocity velocity, Acceleration acceleration = Acceleration{0.0}) const = 0;
};

} // namespace rz