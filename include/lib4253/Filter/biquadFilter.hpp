#pragma once
#include "main.h"

class Biquad{
  double prevInput[2];
  double prevOutput[2];
  double a1, a2, b0, b1, b2; // a0 defaulted to 0
  double sample, cutoff, initVal;

  enum state{
    HIGHPASS, LOWPASS
  };

  Biquad(Biquad::state type, double sampleFreq, double cutoffFreq, double initValue);
  void initialize();
  double filter(double input);
};
