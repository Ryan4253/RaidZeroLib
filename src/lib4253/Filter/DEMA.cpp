#include "main.h"

DemaFilter::DemaFilter(){
    alpha = 1, beta = 1;
    reset();
}

DemaFilter::DemaFilter(double a, double b){
    alpha = a, beta = b;
    reset();
}

void DemaFilter::setGain(double a, double b){
    alpha = a, beta = b;
}

double DemaFilter::getOutput(){
    return outputS + outputB;
}

double DemaFilter::filter(double input){
    outputS = alpha * input + (1 - alpha) * (prevOutputS + prevOutputB);
    outputB = beta * (outputS - prevOutputS) + (1 - beta) * (prevOutputB);
    prevOutputS = outputS;
    prevOutputB = outputB;

    return outputS + outputB;
}

void DemaFilter::reset(){
    outputS = outputB = prevOutputS = prevOutputB = 0;
}
