#pragma once
#include "RaidZeroLib/api/Control/Feedforward/FeedforwardController.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace rz {
using namespace okapi;

template <isRQuantity Distance> class SimpleMotorFeedforward : public FeedforwardController<Distance> {
    public:
    using typename FeedforwardController<Distance>::Velocity;
    using typename FeedforwardController<Distance>::Acceleration;

    SimpleMotorFeedforward(double kS, double kV, double kA, double kD);

    SimpleMotorFeedforward(double kS, double kV, double kA);

    double calculate(Velocity velocity, Acceleration acceleration = Acceleration{0.0}) const override;

    protected:
    double kS, kV, kA, kD;
};

template class SimpleMotorFeedforward<QLength>;
template class SimpleMotorFeedforward<QAngle>;

} // namespace rz