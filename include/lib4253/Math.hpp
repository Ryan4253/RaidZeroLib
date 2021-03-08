#include "main.h"

namespace Math{
    double degToRad(double deg);
    double radToDeg(double rad);
    double tickToInch(double tick);
    double inchToTick(double inch);
    double cubicControl(double power);

    double wrapAngle360(double angle);
    double wrapAngle180(double angle);
    double wrapAngle90(double angle);

    Vector toPolar(Vector cart);
    Vector toCart(Vector polar);
};

const double WHEEL_CIRCUM = 2.75 * M_PI;
const double LDIST = 3.389;
const double RDIST = 3.389;
const double MDIST = 5.748;
