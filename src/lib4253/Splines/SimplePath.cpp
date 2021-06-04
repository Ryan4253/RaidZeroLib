#include "lib4253/Splines/SimplePath.hpp"
namespace lib4253{

/////////////////////////////// PATH /////////////////////////////////

SimplePath::SimplePath(){

}

SimplePath::SimplePath(Point2D v){
    rawPoint.push_back(v);
}

SimplePath::SimplePath(std::vector<Point2D> v){
    for(int i = 0; i < v.size(); i++){
        rawPoint.push_back(v[i]);
    }
    generatePath(0, 0, 0);
}

void SimplePath::injectPoint(Point2D v){
    rawPoint.push_back(v);
}

void SimplePath::injectPoint(std::vector<Point2D> v){
    for(int i = 0; i < v.size(); i++){
        rawPoint.push_back(v[i]);
    }
}

void SimplePath::populatePath(){
    waypoint.clear();

    for(int i = 0; i < rawPoint.size()-1; i++){
        Point2D diff = rawPoint[i+1]-rawPoint[i];
        int cnt = ceil(diff.mag() / spacing);
        Point2D inc = (diff.normalize())*(spacing);

        for(int j = 0; j < cnt; j++){
            waypoint.push_back(rawPoint[i]+(inc*j));
        }
    }

    waypoint.push_back(rawPoint.back());
}

void SimplePath::smoothPath(double a, double b, double tolerance){
    double change = tolerance;
    std::vector<Point2D> newPath = waypoint;

    while(change >= tolerance){
        change = 0;
        for(int i = 1; i < waypoint.size()-1; i++){
            Point2D aux = newPath[i];

            newPath[i].x = newPath[i].x + a * (waypoint[i].x - newPath[i].x) + b * (newPath[i-1].x +
            newPath[i+1].x - (2.0 * newPath[i].x));

            newPath[i].y = newPath[i].y + a * (waypoint[i].y - newPath[i].y) + b * (newPath[i-1].y +
            newPath[i+1].y - (2.0 * newPath[i].y));

            change += abs(aux.x + aux.y - newPath[i].x - newPath[i].y);
        }   
    }   
    waypoint = newPath;
}

void SimplePath::updateDistance(){
    distance.push_back(0);
    for(int i = 1; i < waypoint.size(); i++){
        distance.push_back(distance[i-1]+ waypoint[i].distanceTo(waypoint[i-1])) ;
    }
}

void SimplePath::updateCurvature(){
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

void SimplePath::generatePath(double a, double b, double tolerance){
    clearPath();
    populatePath();
    smoothPath(a, b, tolerance);
    updateDistance();
    updateCurvature();
}

void SimplePath::clearPath(){
    rawPoint.clear();
    waypoint.clear();
    distance.clear();
    curvature.clear();
}

Point2D SimplePath::getWaypoint(int index){
    return waypoint[index];
}

double SimplePath::getDistance(int index){
    if(index > distance.size()){
        return 0;
    }
    else{
        return distance[index];
    }
}

double SimplePath::getCurvature(int index){
    if(index > curvature.size()){
        return 0;
    }
    else{
        return curvature[index];
    }
}

int SimplePath::getSize(){
    return waypoint.size();
}
}