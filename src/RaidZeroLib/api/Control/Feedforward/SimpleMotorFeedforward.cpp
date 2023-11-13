#include "RaidZeroLib/api/Control/Feedforward/SimpleMotorFeedforward.hpp"

namespace rz {

template <isRQuantity Distance>
SimpleMotorFeedforward<Distance>::Gains::Gains(double kS, double kV, double kA, double kD)
    : kS(kS), kV(kV), kA(kA), kD(kD) {
}

template <isRQuantity Distance>
SimpleMotorFeedforward<Distance>::Gains::Gains(double kS, double kV, double kA)
    : SimpleMotorFeedforward::Gains(kS, kV, kA, kA) {
}

template <isRQuantity Distance>
bool SimpleMotorFeedforward<Distance>::Gains::operator==(const Gains& rhs) const {
    return std::abs(kS - rhs.kS) < 1e-6 && std::abs(kV - rhs.kV) < 1e-6 && std::abs(kA - rhs.kA) < 1e-6 &&
           std::abs(kD - rhs.kD) < 1e-6;
}

template <isRQuantity Distance>
bool SimpleMotorFeedforward<Distance>::Gains::operator!=(const Gains& rhs) const {
    return !operator==(rhs);
}

template <isRQuantity Distance>
SimpleMotorFeedforward<Distance>::SimpleMotorFeedforward(const Gains& gains) : gains(gains) {
}

template <isRQuantity Distance>
double SimpleMotorFeedforward<Distance>::calculate(Velocity velocity, Acceleration acceleration) const {
    if (acceleration.getValue() > 0) {
        return gains.kS * sgn(velocity.getValue()) + gains.kV * velocity.getValue() +
               gains.kA * acceleration.getValue();
    }

    return gains.kS * sgn(velocity.getValue()) + gains.kV * velocity.getValue() + gains.kD * acceleration.getValue();
}

template <isRQuantity Distance>
void SimpleMotorFeedforward<Distance>::setGains(const Gains& gains) {
    this->gains = gains;
}

template <isRQuantity Distance>
typename SimpleMotorFeedforward<Distance>::Gains SimpleMotorFeedforward<Distance>::getGains() const {
    return gains;
}

} // namespace rz