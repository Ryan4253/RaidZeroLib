#include "lib4253/Chassis/Drive.hpp"
#include "lib4253/Chassis/Odometry.hpp"

namespace lib4253{

class OdomController{
    public:
    OdomController() ;
    ~OdomController() = default;  

    void moveToPoint(const Point2D& target, double turnScale, Settler settler = Settler::getDefaultSettler());
    void turnToAngle(const double& angle, Settler settler = Settler::getDefaultSettler());
    void facePoint(const Point2D& target, Settler settler = Settler::getDefaultSettler());


    private:
    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<Odometry> odom;
    std::unique_ptr<PID> drivePID;
    std::unique_ptr<PID> turnPID;
    std::unique_ptr<PID> anglePID;
    std::unique_ptr<SlewController> driveSlew;
    const okapi::QLength driveRadius;
};

}