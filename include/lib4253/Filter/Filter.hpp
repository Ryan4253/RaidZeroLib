#pragma once
#include "main.h"

class FilterBase{
  public:
    virtual void reset() = 0;
    virtual double filter(double input) = 0;
};
