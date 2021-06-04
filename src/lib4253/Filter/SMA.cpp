#include "lib4253/Filter/SMA.hpp"
namespace lib4253{

template<int N>
double SmaFilter<N>::filter(double input){
    total += input;
    if(value.size() >= N){
        total -= value.front();
        value.pop();
    }
    value.push(input);

    output = total / value.size();
    return output;
}

template<int N>
double SmaFilter<N>::getOutput(){
    return output;
}

template<int N>
void SmaFilter<N>::reset(){
    while(!value.empty()){
        value.pop();
    }
    total = 0;
    output = 0;
}
}