#include "MotorVelocityController.hpp"
namespace lib4253{

MotorFFController::Gains::Gains(double ikVel, double ikAcc, double ikDec, double ikP_Pos, double ikP_Vel)
    : kVel(ikVel), kAcc(ikAcc), kDec(ikDec), kP_Pos(ikP_Pos), kP_Vel(ikP_Vel) {}

bool MotorFFController::Gains::operator==(const Gains & rhs) const {
  return (kVel == rhs.kVel) && (kAcc == rhs.kAcc) && (kDec == rhs.kDec) && (kP_Pos == rhs.kP_Pos) && (kP_Vel == rhs.kP_Vel);
}

bool MotorFFController::Gains::operator!=(const MotorFFController::Gains &rhs) const {
    return !(*this == rhs);
}

MotorFFController::MotorFFController(double ikVel, double ikAcc, double ikDec, double ikP_Pos, double ikP_Vel):
    MotorFFController(Gains(ikVel, ikAcc, ikDec, ikP_Pos, ikP_Vel)){}

MotorFFController::MotorFFController(const Gains& iGain):
    gains(iGain){}

double MotorFFController::step(double iPosition, double iVelocity, double iAcceleration, double iCurrentPos, double iCurrentVel){
    if(iAcceleration > 0){
        return power = (gains.kVel * iVelocity + gains.kAcc * iAcceleration + gains.kP_Pos * (iPosition - iCurrentPos) + gains.kP_Vel * (iVelocity - iCurrentVel));
    }
    else{
        return power = (gains.kVel * iVelocity + gains.kDec * iAcceleration + gains.kP_Pos * (iPosition - iCurrentPos) + gains.kP_Vel * (iVelocity - iCurrentVel));
    }
}

double MotorFFController::step(QLength iPosition, QSpeed iVelocity, QAcceleration iAcceleration, QLength iCurrentPos, QSpeed iCurrentVel){
    return step(iPosition.convert(foot), iVelocity.convert(ftps), iAcceleration.convert(ftps2), iCurrentPos.convert(foot), iCurrentVel.convert(ftps));
}

double MotorFFController::getTargetPower() const{
    return power;
}

MotorFFController::Gains MotorFFController::getGains() const{
    return gains;
}

void MotorFFController::setGains(const Gains& iGain){
    gains = iGain;
}

}