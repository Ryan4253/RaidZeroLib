#include "main.h"

class Odom{
  private:
    static Pose globalPos;
    static double lPrev, rPrev, mPrev;

  public:
    Odom(Pose initial);
    static void updatePos(void *ptr);

    static void resetSensors();
    static void resetState();
    static void reset();

    static void setPos(Pose newPos);
    static void setX(double newx);
    static void setY(double newy);
    static void setAngle(double theta);

    static Pose getPos();
    static double getX();
    static double getY();
    static double getAngleRad();
    static double getAngleDeg();
};
