#include "lib4253/Controller/BangBang.hpp"
namespace lib4253{

BangBang::BangBang(const BangBangGain& gain){
    this->gain = gain;
}

void BangBang::initialize(){
    error = 0;
}

void BangBang::reset(){
    error = 0;
}

double BangBang::step(const double& val){
    error = val;
    if(error < gain.range){
        return output = gain.targetPower;
    }
    else if(error > gain.range){
        return output = gain.highPower;
    }
    else{
        return output = gain.lowPower;
    }
}

}
