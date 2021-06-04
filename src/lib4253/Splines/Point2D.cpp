#include "main.h"
namespace lib4253{

////////////////////////////// Point2D ////////////////////////////////

Point2D::Point2D(const double& _x, const double& _y){
    this->x = _x, this->y = _y;
}

Point2D& Point2D::operator=(const Point2D& rhs){
    this->x = rhs.x;
    this->y = rhs.y;
    return *this;
}

bool Point2D::operator==(const Point2D& rhs) const{
    return (this->x == rhs.x && this->y == rhs.y);
}
    
bool Point2D::operator!=(const Point2D& rhs) const{
    return (this->x != rhs.x || this->y == rhs.y);
}

Point2D Point2D::operator+(const Point2D& rhs) const{
    Point2D result({this->x + rhs.x, this->y + rhs.y});
    return result;
}

Point2D& Point2D::operator+=(const Point2D& rhs){
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

Point2D Point2D::operator-(const Point2D& rhs) const{
    Point2D result({this->x - rhs.x, this->y - rhs.y});
    return result;
}

Point2D& Point2D::operator-=(const Point2D& rhs){
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

double Point2D::operator*(const Point2D& rhs) const{
    return this->x * rhs.x + this->y * rhs.y;
}

Point2D Point2D::operator*(const double& rhs) const{
    Point2D result({this->x * rhs, this->y * rhs});
    return result;
}

Point2D& Point2D::operator*=(const double& rhs){
    this->x *= rhs;
    this->y *= rhs;
    return *this;
}

Point2D Point2D::operator/(const double& rhs) const{
    Point2D result({this->x / rhs, this->y / rhs});
    return result;
}

Point2D& Point2D::operator/=(const double& rhs){
    this->x /= rhs;
    this->y /= rhs;
    return *this;
}

double Point2D::distanceTo(const Point2D& target) const{
    double deltax = this->x - target.x;
    double deltay = this->y - target.y;

    return sqrt(deltax * deltax + deltay * deltay);
}

double Point2D::angleTo(const Point2D& target) const{
    double deltax = this->x - target.x;
    double deltay = this->y - target.y;

    return (deltax == 0 && deltay == 0) ? 0 : Math::degToRad(atan2(deltax, deltay));
}

Point2D Point2D::normalize() const{
    double magnitude = this->mag();
    return {this->x / magnitude, this->y / magnitude};
}

double Point2D::mag() const{
    return sqrt(this->x * this->x + this->y * this->y);
}

/////////////////////////////// POSE /////////////////////////////////

Pose2D::Pose2D(const double& _x, const double& _y, const double& _heading){
    this->x = _x, this->y = _y, this->theta = Math::degToRad(_heading);
}

Pose2D::Pose2D(const Point2D& coord, const double& _heading){
    this->x = coord.x, this->y = coord.y, this->theta = Math::degToRad(_heading);
}

Pose2D& Pose2D::operator=(const Pose2D& rhs){
    this->x = rhs.x;
    this->y = rhs.y;
    this->theta = rhs.theta;
    return *this;
}

bool Pose2D::operator==(const Pose2D& rhs) const{
    return (this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta);
}

bool Pose2D::operator!=(const Pose2D& rhs) const{
    return (this->x != rhs.x || this->y != rhs.y || this->theta != rhs.theta);
}

Pose2D Pose2D::operator+(const Pose2D& rhs) const{
    Pose2D result = {this->x + rhs.x, this->y + rhs.y, this->theta + rhs.theta};
    return result;
}

Pose2D& Pose2D::operator+=(const Pose2D& rhs){
    this->x += rhs.x, this->y += rhs.y, this->theta += rhs.theta;
    return *this;
}

Pose2D Pose2D::operator-(const Pose2D& rhs) const{
    Pose2D result = {this->x - rhs.x, this->y - rhs.y, this->theta - rhs.theta};
    return result;
}

Pose2D& Pose2D::operator-=(const Pose2D& rhs){
    this->x -= rhs.x, this->y -= rhs.y, this->theta -= rhs.theta;
    return *this;
}

Pose2D Pose2D::operator+(const Point2D& rhs) const{
    Pose2D result({this->x + rhs.x, this->y + rhs.y, this->theta});
    return result;
}

Pose2D& Pose2D::operator+=(const Point2D& rhs){
    this->x += rhs.x, this->y += rhs.y;
    return *this;
}

Pose2D Pose2D::operator-(const Point2D& rhs) const{
    Pose2D result = {this->x - rhs.x, this->y - rhs.y, this->theta};
    return result;
}

Pose2D& Pose2D::operator-=(const Point2D& rhs){
    this->x -= rhs.x, this->y -= rhs.y;
    return *this;
}

Pose2D Pose2D::operator*(const double& rhs) const{
    Pose2D result({this->x * rhs, this->y * rhs, this->theta});
    return result;
}

Pose2D& Pose2D::operator*=(const double& rhs){
    this->x *= rhs, this->y *= rhs;
    return *this;
}

Pose2D Pose2D::operator/(const double& rhs) const{
    Pose2D result({this->x / rhs, this->y / rhs, this->theta});
    return result;
}

Pose2D& Pose2D::operator/=(const double& rhs){
    this->x /= rhs, this->y /= rhs;
    return *this;
}

double Pose2D::angleTo(const Point2D& target) const{
    double deltax = this->x - target.x;
    double deltay = this->y - target.y;
    double angle = (deltax == 0 && deltay == 0) ? 0 : atan2(deltax, deltay);
    return Math::wrapAngle180(Math::radToDeg(angle - this->theta));
}

double Pose2D::angleTo(const Pose2D& target) const{
    double result = Math::radToDeg(this->theta - target.theta);
    return Math::wrapAngle180(result);
}

double Pose2D::distanceTo(const Pose2D& target) const{
    return this->distanceTo({target.x, target.y});
}

Point2D Pose2D::closest(const Point2D& target) const{
    Point2D current = {this->x, this->y};
    Point2D heading = {sin(this->theta), cos(this->theta)};
    Point2D n = heading.normalize();
    Point2D v = target-current;
    double d = n*v;

    return (current)+((n*d));
}

Pose2D Pose2D::normalize() const{
    double magnitude = this->mag();
    return {this->x / magnitude, this->y / magnitude, this->theta};
}

Point2D Pose2D::toPoint() const{
    Point2D result({this->x, this->y});
    return result;
}
}