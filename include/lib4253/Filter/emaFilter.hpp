#pragma once
#include "main.h"

class emaFilter{
  private:
      double alpha, output = 0, prevOutput = 0;
      bool run = false;

  public:
      emaFilter(double a);
      emaFilter();
      void setGain(double a);
      double filter(double input);
      void reset();
      double getOutput();
};
