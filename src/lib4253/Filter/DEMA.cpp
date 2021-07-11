#include "lib4253/Filter/DEMA.hpp"
namespace lib4253{

DemaFilter::DemaFilter(){
    alpha = 1, beta = 1;
    reset();
}

DemaFilter::DemaFilter(const double& a, const double& b){
    alpha = a, beta = b;
    reset();
}

void DemaFilter::setGain(const double& a, const double& b){
    alpha = a, beta = b;
}

void DemaFilter::initialize(){
    outputS = outputB = prevOutputS = prevOutputB = 0;

}

void DemaFilter::reset(){
    initialize();
}

double DemaFilter::filter(const double& input) {
    outputS = alpha * input + (1 - alpha) * (prevOutputS + prevOutputB);
    outputB = beta * (outputS - prevOutputS) + (1 - beta) * (prevOutputB);
    prevOutputS = outputS;
    prevOutputB = outputB;

    return outputS + outputB;
}

double DemaFilter::getOutput() const {
    return outputS + outputB;
}
}