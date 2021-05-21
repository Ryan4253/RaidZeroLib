/**
 * @file SimplePath.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Simple Path
 * @version 0.1
 * @date 2021-05-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include "main.h"

<<<<<<< HEAD
/**
 * @brief 4253B custom programming library
 * 
 */
namespace lib4253{

  /**
   * @brief Simple path structure
   * 
   */
  struct SimplePath{
    private:
=======
struct SimplePath{
  private:
>>>>>>> e28f0a1052c337f3570dcdcd9a98ec1947c8505c
    std::vector<Point2D> rawPoint;
    std::vector<Point2D> waypoint;
    std::vector<double> distance;
    std::vector<double> curvature;
    double spacing = 0.5;

    /**
     * @brief Smooths given path
     * 
     * @param a 
     * @param b 
     * @param tolerance 
     */
    void smoothPath(double a, double b, double tolerance);

    /**
     * @brief Smooths given path with qunitic spline
     * 
     */
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
    double getDistance(int index);
    double getCurvature(int index);
    int getSize();
<<<<<<< HEAD
  };

}
=======
};
>>>>>>> e28f0a1052c337f3570dcdcd9a98ec1947c8505c
