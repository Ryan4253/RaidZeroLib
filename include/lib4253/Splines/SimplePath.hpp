#pragma once
#include "main.h"

struct SimplePath{
  private:
    std::vector<Point2D> rawPoint;
    std::vector<Point2D> waypoint;
    std::vector<double> distance;
    std::vector<double> curvature;
    double spacing = 0.5;

    void smoothPath(double a, double b, double tolerance);
    void smoothPathQuinticSpline();
    void populatePath();
    void updateDistance();
    void updateCurvature();

  public:
    SimplePath();
    SimplePath(std::vector<Point2D> v);
    SimplePath(Point2D v);

    void injectPoint(Point2D v);
    void injectPoint(std::vector<Point2D> v);

    void generatePath(double a, double b, double tolerance);
    void clearPath();

    Point2D getWaypoint(int index);
    std::vector<Point2D> getWaypoint();
    double getDistance(int index);
    std::vector<double> getDistance();
    double getCurvature(int index);
    std::vector<double> getCurvature();
};
