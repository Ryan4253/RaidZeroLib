#include "main.h"

SmaFilter::SmaFilter(){
    maxSize = 1000;
}

SmaFilter::SmaFilter(int size){
    maxSize = size;
}

double SmaFilter::filter(double input){
    total += input;
    if(value.size() >= maxSize){
        total -= value.front();
        value.pop();
    }
    value.push(input);

    output = total / value.size();
    return output;
}

double SmaFilter::getOutput(){
    return output;
}

void SmaFilter::reset(){
    while(!value.empty()){
        value.pop();
    }
    total = 0;
    output = 0;
}

void SmaFilter::setMaxSize(int size){
    maxSize = size;
}
