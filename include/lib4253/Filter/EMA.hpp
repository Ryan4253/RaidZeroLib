#pragma once
#include "main.h"

class EmaFilter: public Filter{
  private:
      double alpha, output = 0, prevOutput = 0;
      bool run = false;

  public:
      EmaFilter();
      EmaFilter(double a);

      void setGain(double a);
      double getOutput();

      double filter(double input);
      void reset();
};
