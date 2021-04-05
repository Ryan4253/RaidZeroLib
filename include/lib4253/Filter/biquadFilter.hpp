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

  Biquad(double a1_, double a2_, double b0_, double b1_, double b2_, double sampleFreq, double cutoffFreq, double initValue);
  void initialize();
  double filter(double input);
};
