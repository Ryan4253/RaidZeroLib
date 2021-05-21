#pragma once
#include "main.h"

class Filter{
  public:
    virtual void reset() = 0;
    virtual double filter(double input) = 0;
};
