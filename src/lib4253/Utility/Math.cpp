#include "main.h"

double Math::degToRad(double deg){
  return deg * M_PI / 180;
}

double Math::radToDeg(double rad){
  return rad * 180 / M_PI;
}

double Math::tickToInch(double tick){
  return tick * WHEEL_CIRCUM / 360;
}

double Math::inchToTick(double inch){
  return (inch * (360/WHEEL_CIRCUM));
}

double Math::cubicControl(double power){
  return power * power * power / 16129;
}

Vector Math::toPolar(Vector cart){
  double mag = sqrt(cart.x * cart.x + cart.y * cart.y);
  double angle = atan2(cart.y, cart.y);

  return {mag, angle};
}

Vector Math::toCart(Vector polar){
  double mag = polar.x, angle = polar.y;
  return {mag * cos(angle), mag * sin(angle)};
}

double Math::wrapAngle360(double angle){
  return angle - 360.0 * (std::floor(angle * (1.0 / 360.0)));
}

double Math::wrapAngle180(double angle){
  return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

double Math::wrapAngle90(double angle){
  angle = wrapAngle180(angle);
  return wrapAngle180(angle + (abs(angle) > 90) * 180);
}

double Math::clamp(double val, double mn, double mx){
  val = fmax(mn, val);
  val = fmin(mx, val);

  return val;
}
