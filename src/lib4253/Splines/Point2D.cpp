#include "main.h"

////////////////////////////// Point2D ////////////////////////////////

Point2D::Point2D(){
    x = 0, y = 0;
}

Point2D::Point2D(double a, double b){
    x = a, y = b;
}

Point2D Point2D::operator=(Point2D a){
    return {a.x, a.y};
}

Point2D Point2D::operator+(Point2D a){
    return {x + a.x, y + a.y};
}

Point2D Point2D::operator-(Point2D a){
    return {x - a.x, y - a.y};
}

Point2D Point2D::operator*(double a){
    return {x * a, y * a};
}

double Point2D::operator*(Point2D a){
    return x * a.x + y * a.y;
}

Point2D Point2D::operator/(double a){
    return {x / a, y / a};
}

Point2D Point2D::normalize(){
    double mag = sqrt(x * x + y * y);
    return {x / mag, y / mag};
}

double Point2D::mag(){
    return sqrt(x * x + y * y);
}

double Point2D::distanceTo(Point2D target){
    double deltax = x - target.x;
    double deltay = y - target.y;

    return sqrt(deltax * deltax + deltay * deltay);
}

double Point2D::angleTo(Point2D target){
    double deltax = x - target.x;
    double deltay = y - target.y;

    return (deltax == 0 && deltay == 0) ? 0 : atan2(deltax, deltay);
}

/////////////////////////////// POSE /////////////////////////////////

Pose2D::Pose2D(){
    Point2D(0, 0);
    angle = 0;
}

Pose2D::Pose2D(double a, double b){
    Point2D(a, b);
    angle = 0;
}

Pose2D::Pose2D(double a, double b, double theta){
    Point2D(a, b);
    angle = theta;
}

Point2D Pose2D::closest(Point2D target){
    Point2D current = {x, y};
    Point2D heading = {sin(angle), cos(angle)};
    Point2D n = heading.normalize();
    Point2D v = target-current;
    double d = n*v;

    return (current)+((n*d));
}

double Pose2D::angleTo(Point2D target){
    double deltax = x - target.x;
    double deltay = y - target.y;

    double ang = (deltax == 0 && deltay == 0) ? 0 : atan2(deltax, deltay);
    ang -= angle;
    return Math::wrapAngle180(Math::radToDeg(ang));
}

double Pose2D::angleTo(Pose2D target){
    double ang = angle - target.angle;
    return Math::wrapAngle180(ang);
}
