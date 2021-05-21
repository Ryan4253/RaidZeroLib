#pragma once
#include "main.h"

struct Point2D{
  double x, y;

  Point2D();
  Point2D(double a, double b);

  Point2D operator=(Point2D a);
  Point2D operator+(Point2D a);
  Point2D operator-(Point2D a);
  Point2D operator*(double a);
  double operator*(Point2D a);
  Point2D operator/(double a);

  double distanceTo(Point2D target);
  double angleTo(Point2D target);
  Point2D normalize();
  double mag();
};

struct Pose2D:Point2D{
  double angle;

  Pose2D();
  Pose2D(double a, double b);
  Pose2D(double a, double b, double theta);

  Point2D closest(Point2D target);
  double angleTo(Point2D target);
  double angleTo(Pose2D target);
};

