#pragma once
#include "main.h"
namespace lib4253{

class Filter{
  public:
    virtual void reset() = 0;
    virtual double filter(double input) = 0;
};

}
