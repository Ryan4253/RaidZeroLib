#pragma once

class BangBang{
  double highPower, lowPower, targetVel;
  BangBang(double h, double l, double t);
  void setTargetVel(double t);
  double step(double v);
};
