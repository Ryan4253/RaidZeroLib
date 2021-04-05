#pragma once
#include "main.h"

class avgFilter{
  std::queue<double> value; // ngl i was thinking about coding a segment tree
  int maxSize;
  double total, output;

  avgFilter();
  avgFilter(int size);

  double filter(double input);
  double getOutput();

  void setMaxSize(int size);
  void reset();
};
