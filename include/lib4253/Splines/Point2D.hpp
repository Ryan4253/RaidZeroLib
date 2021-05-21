/**
 * @file Point2D.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief 2D Point structs
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
   * @brief 2D point structure
   * 
   */
  struct Point2D{
    double x, y;

    /**
     * @brief Construct a new Point 2D object
     * 
     */
    Point2D();

    /**
     * @brief Construct a new Point 2D object
     * 
     * @param a x value
     * @param b y value
     */
    Point2D(double a, double b);

    
    Point2D operator=(Point2D a);
    Point2D operator+(Point2D a);
    Point2D operator-(Point2D a);
    Point2D operator*(double a);
    double operator*(Point2D a);
    Point2D operator/(double a);
=======
struct Point2D{
  double x, y;
>>>>>>> e28f0a1052c337f3570dcdcd9a98ec1947c8505c

    /**
     * @brief Distance from initial point to target point
     * 
     * @param target target point
     * @return distance between the 2 points
     */
    double distanceTo(Point2D target);

    /**
     * @brief Angle from initial point to target point
     * 
     * @param target target point
     * @return angle between the 2 points
     */
    double angleTo(Point2D target);
    Point2D normalize();
    double mag();
  };
  
  /**
   * @brief 2D position structure - inherited from 2D point structure
   * 
   */
  struct Pose2D : Point2D{
    double angle;

    /**
     * @brief Construct a new Pose 2D object
     * 
     */
    Pose2D();

    /**
     * @brief Construct a new Pose 2 D object
     * 
     * @param a x value
     * @param b y value
     */
    Pose2D(double a, double b);

    /**
     * @brief Construct a new Pose 2 D object
     * 
     * @param a x value
     * @param b y value
     * @param theta angle
     */
    Pose2D(double a, double b, double theta);

    
    Point2D closest(Point2D target);
    double angleTo(Point2D target);
    double angleTo(Pose2D target);
  };

