#pragma once
#include "RaidZeroLib/api/Control/Feedforward/FeedforwardController.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace rz {
using namespace okapi;

template <isRQuantity Distance>
class SimpleMotorFeedforward : public FeedforwardController<Distance> {
    public:
    using typename FeedforwardController<Distance>::Velocity;
    using typename FeedforwardController<Distance>::Acceleration;

    struct Gains {
        double kS{0.0}, kV{0.0}, kA{0.0}, kD{0.0};
        Gains() = default;
        ~Gains() = default;
        Gains(double kS, double kV, double kA, double kD);
        Gains(double kS, double kV, double kA);
        bool operator==(const Gains& rhs) const;
        bool operator!=(const Gains& rhs) const;
    };

    SimpleMotorFeedforward(const Gains& gains);

    double calculate(Velocity velocity, Acceleration acceleration = Acceleration{0.0}) const override;

    void setGains(const Gains& gains);

    Gains getGains() const;

    protected:
    Gains gains;
};

template class SimpleMotorFeedforward<QLength>;
template class SimpleMotorFeedforward<QAngle>;

} // namespace rz