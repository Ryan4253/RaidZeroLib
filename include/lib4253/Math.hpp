#include "main.h"

struct Vector{
  std::atomic<double> x;
  std::atomic<double> y;

  Vector(double a, double b){
    x = a, y = b;
  }
  Vector(const Vector &p2){
    x = (double)p2.x, y = (double)p2.y;
  }

  double distanceTo(Vector target);
  double angleTo(Vector target);
  Vector add(Vector a);
  Vector sub(Vector a);
  Vector mult(Vector a);
  Vector div(Vector a);
  Vector normalize();
  Vector scale(double factor);
  double mag();
  double dot(Vector a);
};

struct Pose{
  std::atomic<double> x;
  std::atomic<double> y;
  std::atomic<double> angle;

  Pose(double a, double b){
    x = a, y = b, angle = 0;
  }
  Pose(const Pose &p2){
    x = (double)p2.x, y = (double)p2.y, angle = (double)p2.angle;
  }
  Pose(double a, double b, double theta){
    x = a, y = b, angle = theta;
  }

  Vector closest(Vector target);
  Vector toVector();
  double distanceTo(Vector target);
  double angleTo(Vector target);
};

namespace Math{
    double degToRad(double deg);
    double radToDeg(double rad);
    double tickToInch(double tick);
    double inchToTick(double inch);
    double cubicControl(double power);

    double wrapAngle360(double angle);
    double wrapAngle180(double angle);
    double wrapAngle90(double angle);
    
    Vector toPolar(Vector cart);
    Vector toCart(Vector polar);
};

const double WHEEL_CIRCUM = 2.75 * M_PI;
const double LDIST = 3.389;
const double RDIST = 3.389;
const double MDIST = 5.748;
