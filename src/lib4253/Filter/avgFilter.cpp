#include "main.h"
#include "avgFilter.hpp"

avgFilter::avgFilter(){
  maxSize = 1000;
}

avgFilter::avgFilter(int size){
  maxSize = size;
}

double avgFilter::filter(double input){
  total += input;
  if(value.size() >= maxSize){
    total -= value.front();
    value.pop();
  }
  value.push(input);

  output = total / value.size();
  return output;
}

double avgFilter::getOutput(){
  return output;
}

void avgFilter::reset(){
  while(!value.empty()){
    value.pop();
  }
  total = 0;
  output = 0;
}

void avgFilter::setMaxSize(int size){
  maxSize = size;
}
