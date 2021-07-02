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
#include "lib4253/Utility/Math.hpp"

namespace lib4253{
/**
 * @brief 2D point / vector class
 *
 */
class Point2D{
    public:
    double x{0}, y{0};

    /** 
     * @brief Construct a new Point 2D object
     *
     * @param _x x value
     * @param _y y value
     */
    Point2D(const double& _x = 0, const double& _y = 0);

    /**
     * @brief Destruct a Point 2D object
     *
     */
    virtual ~Point2D() = default;

    /**
     * @brief Point 2D operators
     *
     * @param rhs right hand side of the operation
     */
    Point2D& operator=(const Point2D& rhs);
    bool operator==(const Point2D& rhs) const;
    bool operator!=(const Point2D& rhs) const;
    Point2D operator+(const Point2D& rhs) const;
    Point2D& operator+=(const Point2D& rhs);
    Point2D operator-(const Point2D& rhs) const;
    Point2D& operator-=(const Point2D& rhs) ;
    double operator*(const Point2D& rhs) const;

    /**
     * @brief Point 2D scalars
     *
     * @param rhs right hand side of the operation
     */
    Point2D operator*(const double& rhs) const;
    Point2D& operator*=(const double& rhs);
    Point2D operator/(const double& rhs) const;
    Point2D& operator/=(const double& rhs);

    /**
     * @brief Distance from initial point to target point
     *
     * @param target target point
     * @return distance between the 2 points
     */
    virtual double distanceTo(const Point2D& target) const;

    /**
     * @brief Angle from initial point to target point
     *
     * @param target target point
     * @return Angle between the 2 points
     */
    virtual double angleTo(const Point2D& target) const;

    /**
     * @brief normalizes the vector
     *
     * @return The normalized vector
     */
    Point2D normalize() const;

    /**
     * @brief Magnitude of the vector
     * 
     * @return The magnitude of the vector
     */
    double mag() const;

    // conversion between cartestian and polar coordinates
    static Point2D toPolar(const Point2D& cart);
    static Point2D toCart(const Point2D& polar);
};

}

