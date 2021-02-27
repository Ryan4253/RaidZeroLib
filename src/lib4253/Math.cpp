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

double Vector::distanceTo(Vector target){
  double deltax = x - target.x;
  double deltay = y - target.y;

  return sqrt(deltax * deltax + deltay * deltay);
}

double Vector::angleTo(Vector target){
  double deltax = x - target.x;
  double deltay = y - target.y;

  return (deltax == 0 && deltay == 0) ? 0 : atan2(deltax, deltay);
}

Vector Vector::add(Vector a){
  return {x + a.x, y + a.y};
}

Vector Vector::sub(Vector a){
  return {x - a.x, y - a.y};
}

Vector Vector::mult(Vector a){
  return {x * a.x, y * a.y};
}

Vector Vector::div(Vector a){
  return {x / a.x, y / a.y};
}

Vector Vector::normalize(){
  double mag = sqrt(x * x + y * y);
  return {x / mag, y / mag};
}

Vector Vector::scale(double factor){
  return {x * factor, y * factor};
}

double Vector::mag(){
  return sqrt(x * x + y * y);
}

double Vector::dot(Vector a){
  return x * a.x + y * a.y;
}

Vector Pose::closest(Vector target){
  Vector heading = {sin(angle), cos(angle)};
  Vector n = heading.normalize();
  Vector v = target.sub(this->toVector());
  double d = n.dot(v);


  return (this->toVector()).add((n.scale(d)));
}

Vector Pose::toVector(){
  return {x, y};
}

double Pose::distanceTo(Vector target){
  return (this->toVector()).distanceTo(target);
}

double Pose::angleTo(Vector target){
  double angle =((this->toVector()).angleTo(target))-(this->angle);
  return Math::wrapAngle180(Math::radToDeg(angle));
}
