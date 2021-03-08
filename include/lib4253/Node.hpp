#include "main.h"

struct Vector{
  std::atomic<double> x;
  std::atomic<double> y;

  Vector(double a, double b){
    x = a, y = b;
  }
  Vector(const Vector &p2){
    x = (double)p2.x, y = (double)p2.y;
  }

  Vector operator+(Vector a);
  Vector operator+(std::initializer_list<int>);
  Vector operator-(Vector a);
  Vector operator*(Vector a);
  Vector operator*(double a);
  Vector operator/(Vector a);


  double distanceTo(Vector target);
  double angleTo(Vector target);
  Vector normalize();
  double mag();
  double dot(Vector a);


};

struct Pose{
  std::atomic<double> x;
  std::atomic<double> y;
  std::atomic<double> angle;

  Pose(double a, double b){
    x = a, y = b, angle = 0;
  }
  Pose(const Pose &p2){
    x = (double)p2.x, y = (double)p2.y, angle = (double)p2.angle;
  }
  Pose(double a, double b, double theta){
    x = a, y = b, angle = theta;
  }

  Vector closest(Vector target);
  Vector toVector();
  double distanceTo(Vector target);
  double angleTo(Vector target);
};

struct Path{
  private:
    std::vector<Vector> rawPoint;
    std::vector<Vector> waypoint;
    std::vector<double> distance;
    std::vector<double> curvature;
    double spacing = 0.5;

    void smoothPath(double a, double b, double tolerance);
    void smoothPathQuinticSpline();
    void populatePath();
    void updateDistance();
    void updateCurvature();

  public:
    Path(std::vector<Vector> v);
    Path(Vector v);

    void injectPoint(Vector v);
    void injectPoint(std::vector<Vector> v);

    void generatePath(double a, double b, double tolerance);
    void clearPath();

    std::vector<Vector> getPath();
    Vector getWaypoint(int index);
    double getDistance(int index);
    double getCurvature(int index);
};
