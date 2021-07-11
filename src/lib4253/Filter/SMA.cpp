#include "lib4253/Filter/SMA.hpp"
namespace lib4253{

template<int N>
void SmaFilter<N>::initialize(){
    while(!value.empty()){
        value.pop();
    }
    total = 0;
    output = 0;
}

template<int N>
void SmaFilter<N>::reset(){
    initialize();
}

template<int N>
double SmaFilter<N>::filter(const double& input){
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
double SmaFilter<N>::getOutput() const {
    return output;
}
}