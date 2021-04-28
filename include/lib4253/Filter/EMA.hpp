#pragma once
#include "main.h"

namespace lib4253{

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

}
