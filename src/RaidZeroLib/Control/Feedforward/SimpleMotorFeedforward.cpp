#include "RaidZeroLib/Control/Feedforward/SimpleMotorFeedforward.hpp"

namespace rz{

template<isRQuantity Distance>
SimpleMotorFeedforward<Distance>::SimpleMotorFeedforward(double kS, double kV, double kA, double kD) 
    : kS(kS), kV(kV), kA(kA), kD(kD){}

template<isRQuantity Distance>
SimpleMotorFeedforward<Distance>::SimpleMotorFeedforward(double kS, double kV, double kA) 
    : SimpleMotorFeedforward(kS, kV, kA, kA){}

template<isRQuantity Distance>
double SimpleMotorFeedforward<Distance>::calculate(Velocity velocity, Acceleration acceleration) const{
    if(acceleration.getValue() > 0){
        return kS * sgn(velocity.getValue()) + kV * velocity.getValue() + kA * acceleration.getValue();
    }

    return kS * sgn(velocity.getValue()) + kV * velocity.getValue() + kD * acceleration.getValue();   
}

}