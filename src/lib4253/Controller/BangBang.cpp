#include "lib4253/Controller/BangBang.hpp"
namespace lib4253{

BangBang::BangBang() {
    this->gain = {0, 0, 0};
}

BangBang::BangBang(const BangBangGain& gain){
    this->gain = gain;
}

void BangBang::setTargetVel(const double& t){
    gain.targetVel = t;
}

double BangBang::step(const double& v) const {
    return v > gain.targetVel ? gain.highPower : gain.lowPower;
}
}