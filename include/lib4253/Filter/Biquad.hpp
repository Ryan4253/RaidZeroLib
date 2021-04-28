#pragma once
#include "main.h"

namespace lib4253{

class BiquadFilter:public Filter{
  double prevInput[2];
  double prevOutput[2];
  double a1, a2, b0, b1, b2; // a0 defaulted to 0
  double sample, cutoff, initVal;

  enum state{
    HIGHPASS, LOWPASS
  };

  BiquadFilter(BiquadFilter::state type, double sampleFreq, double cutoffFreq, double initValue);
  void reset();
  double filter(double input);
};

}
