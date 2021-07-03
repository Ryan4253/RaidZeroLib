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
#include "lib4253/Splines/Geometry/Rotation2D.hpp"

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

class Translation2D{
    public:
    constexpr Translation2D() = default;

    Translation2D(const okapi::QLength& xPos, const okapi::QLength& yPos);

    Translation2D(const okapi::QLength& magnitude, const Rotation2D& angle);

    ~Translation2D() = default;

    okapi::QLength getX() const;

    okapi::QLength getY() const;

    Translation2D operator+(const Translation2D& rhs) const;

    Translation2D operator-(const Translation2D& rhs) const;

    Translation2D operator-() const;

    Translation2D operator*(const double& scalar) const;

    okapi::QArea operator*(const Translation2D& rhs) const;

    Translation2D operator/(const double& scalar) const;

    bool operator==(const Translation2D& rhs) const;

    bool operator!=(const Translation2D& rhs) const;

    okapi::QLength distanceTo(const Translation2D& rhs) const;

    okapi::QAngle angleTo(const Translation2D& target) const;

    okapi::QLength magnitude() const;

    Translation2D rotateBy(const Rotation2D& rhs) const;

    private:
    okapi::QLength x = 0 * okapi::meter;
    okapi::QLength y = 0 * okapi::meter;

};
}

