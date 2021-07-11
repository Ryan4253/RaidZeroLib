#include "lib4253/Filter/Biquad.hpp"

namespace lib4253{

BiquadFilter::BiquadFilter(const BiquadFilter::State& type, const double& sampleFreq, const double& cutoffFreq, const double& initValue = 0){
    setGain(type, sampleFreq, cutoffFreq, initValue);
}

void BiquadFilter::setGain(const BiquadFilter::State& type, const double& sampleFreq, const double& cutoffFreq, const double& initValue){
    double w0 =  2 * M_PI * cutoffFreq / sampleFreq;
    double cosw0 = cos(w0);
    double sinw0 = sin(w0);

    double q = 0.707;
    double alpha = sinw0 / (2 * q);
    double a0;

    switch (type){
        case LOWPASS:
        b0 = (1 - cosw0) / 2;
        b1 = (1 - cosw0);
        b2 = (1 - cosw0) / 2;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;

        case HIGHPASS:
        b0 = (1 + cosw0) / 2;
        b1 = -(1 + cosw0);
        b2 = (1 + cosw0) / 2;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
    }

    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    initVal = initValue;

    prevInput[0] = prevInput[1] = initVal;
    prevOutput[0] = prevOutput[1] = initVal;
}

void BiquadFilter::initialize(){
    prevInput[0] = prevInput[1] = initVal;
    prevOutput[0] = prevOutput[1] = initVal;
    output = initVal;
}

void BiquadFilter::reset(){
    initialize();
}

double BiquadFilter::filter(const double& input) {
    output = b0 * input + b1 * prevInput[0] + b2 * prevInput[1] - a1 * prevOutput[0] - a2 * prevOutput[1];
    prevInput[1] = prevInput[0]; prevInput[0] = input;
    prevOutput[1] = prevOutput[0]; prevOutput[0] = output;
    
    return output;
}

double BiquadFilter::getOutput() const{
    return output;
}
}