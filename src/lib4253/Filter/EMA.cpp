#include "lib4253/Filter/EMA.hpp"
namespace lib4253{

EmaFilter::EmaFilter(const double& a){
    alpha = a;
    reset();
}

EmaFilter::EmaFilter(){
    alpha = 1;
    reset();
}

void EmaFilter::setGain(const double& a){
    alpha = a;
}

double EmaFilter::filter(const double& input){
    if(!run){
        run = true;
        output = prevOutput = input;
    }
    else{
        output = alpha * input + (1 - alpha) * prevOutput;
        prevOutput = output;
    }

    return output;
}

void EmaFilter::reset(){
    output = 0, prevOutput = 0;
    run = false;
}

double EmaFilter::getOutput() const {
    return output;
}
}