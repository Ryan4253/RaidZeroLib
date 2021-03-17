  ㄇ#include "main.h"

////////////////////////////// VECTOR ////////////////////////////////

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

Vector Vector::operator+(Vector a){
  return {x + a.x, y + a.y};
}

Vector Vector::operator+(std::initializer_list<int> s){
  return {x + *s.begin(), y + *s.end()};
}

Vector Vector::operator-(Vector a){
  return {x - a.x, y - a.y};
}

Vector Vector::operator*(Vector a){
  return {x * a.x, y * a.y};
}

Vector Vector::operator*(double a){
  return {x * a, y * a};
}

Vector Vector::operator/(Vector a){
  return {x / a.x, y / a.y};
}

Vector Vector::normalize(){
  double mag = sqrt(x * x + y * y);
  return {x / mag, y / mag};
}

double Vector::mag(){
  return sqrt(x * x + y * y);
}

double Vector::dot(Vector a){
  return x * a.x + y * a.y;
}

/////////////////////////////// POSE /////////////////////////////////

Vector Pose::closest(Vector target){
  Vector heading = {sin(angle), cos(angle)};
  Vector n = heading.normalize();
  Vector v = target-(this->toVector());
  double d = n.dot(v);


  return (this->toVector())+((n*d));
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

/////////////////////////////// PATH /////////////////////////////////

Path::Path(Vector v){
  rawPoint.push_back(v);
}

Path::Path(std::vector<Vector> v){
  for(int i = 0; i < v.size(); i++){
    rawPoint.push_back(v[i]);
  }
  generatePath();
}

void Path::injectPoint(Vector v){
  rawPoint.push_back(v);
}

void Path::injectPoint(std::vector<Vector> v){
  for(int i = 0; i < v.size(); i++){
    rawPoint.push_back(v[i]);
  }
}

void Path::populatePath(){
  waypoint.clear();

  for(int i = 0; i < rawPoint.size()-1; i++){
    Vector diff = rawPoint[i+1]-rawPoint[i];
    int cnt = ceil(diff.mag() / spacing);
    Vector inc = (diff.normalize())*(spacing);

    for(int j = 0; j < cnt; j++){
      waypoint.push_back(rawPoint[i]+(inc*j));
    }
  }

  waypoint.push_back(rawPoint.back());
}

void Path::smoothPath(double a, double b, double tolerance){
  double change = tolerance;
  std::vector<Vector> newPath = waypoint;

  while(change >= tolerance){
    change = 0;
    for(int i = 1; i < waypoint.size()-1; i++){
      Vector aux = newPath[i];

      newPath[i].x = newPath[i].x + a * (waypoint[i].x - newPath[i].x) + b * (newPath[i-1].x +
      newPath[i+1].x - (2.0 * newPath[i].x));

      newPath[i].y = newPath[i].y + a * (waypoint[i].y - newPath[i].y) + b * (newPath[i-1].y +
      newPath[i+1].y - (2.0 * newPath[i].y));

      change += abs(aux.x + aux.y - newPath[i].x - newPath[i].y);
    }
  }

	waypoint = newPath;
}

void Path::updateDistance(){
  distance.push_back(0);
  for(int i = 1; i < waypoint.size(); i++){
    distance.push_back(distance[i-1]+ waypoint[i].distanceTo(waypoint[i-1])) ;
  }
}

void Path::updateCurvature(){
  curvature.push_back(0);

  for(int i = 1; i < waypoint.size()-1; i++){
    double x1 = waypoint[i].x + 0.001, y1 = waypoint[i].y ;
    double x2 = waypoint[i-1].x, y2 = waypoint[i-1].y;
    double x3 = waypoint[i+1].x, y3 = waypoint[i+1].y;

    double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1-x2);
    double k2 = (y1-y2) / (x1-x2);
    double b = 0.5 * ((x2*x2)+(2*x2*k1)+(y2*y2)-(x3*x3)+(2*x3*k1)-(y3 * y3)) / ((x3*k2)-y3+y2-(x2*k2));
    double a = k1 - k2 * b;

    double r = sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
    r = std::isnan(r) ? 0.0000000001 : r;

    curvature.push_back(r);
  }

  curvature.push_back(0);
}

void Path::generatePath(double a, double b, double tolerance){
  clearPath();
  populatePath();
  smoothPath(a, b, tolerance);
  updateDistance();
  updateCurvature();
}

void Path::clearPath(){
  rawPoint.clear();
  waypoint.clear();
  distance.clear();
  curvature.clear();
}

std::vector<Vector> Path::getWaypoint(){
  return waypoint;
}

Vector Path::getWaypoint(int index){
  return waypoint[index];
}

double Path::getDistance(int index){
  if(index > distance.size()){
    return 0;
  }
  else{
    return distance[index];
  }
}

std::vector<double> Path::getDistance(){
  return distance;
}

double Path::getCurvature(int index){
  if(index > curvature.size()){
    return 0;
  }
  else{
    return curvature[index];
  }
}

std::vector<double> Path::getCurvature(){
  return curvature;
}
