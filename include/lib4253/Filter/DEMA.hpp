#include "main.h"

class DemaFilter: public Filter{
  private:
    double alpha, beta;
    double outputS, outputB;
    double prevOutputS, prevOutputB;

  public:
    DemaFilter();
    DemaFilter(double a, double b);

    void setGain(double a, double b);
    double getOutput();

    double filter(double input);
    void reset();
};
