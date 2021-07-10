#include "lib4253/Chassis/Drive.hpp"
#include "lib4253/Chassis/Odometry.hpp"

namespace lib4253{

class OdomController{
    public:
    OdomController(std::shared_ptr<Chassis> iChassis, 
                                   std::shared_ptr<Odometry> iOdometry, 
                                   const okapi::QLength& iAngleCorrectionRadius,
                                   std::unique_ptr<PID> iDrivePID, 
                                   std::unique_ptr<PID> iTurnPID, 
                                   std::unique_ptr<PID> iAnglePID, 
                                   std::unique_ptr<SlewController> iSlew);
                                
    ~OdomController() = default;  

    void moveToPoint(const Point2D& target, const double& turnScale, Settler settler = Settler::getDefaultSettler());
    void moveToX(const QLength& targetX, Settler settler = Settler::getDefaultSettler());
    void moveToY(const QLength& targetY, Settler settler = Settler::getDefaultSettler());
    void turnToAngle(const okapi::QAngle& angle, Settler settler = Settler::getDefaultSettler());
    void turnToPoint(const Point2D& target, Settler settler = Settler::getDefaultSettler());

    private:
    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<Odometry> odom;
    std::unique_ptr<PID> drivePID;
    std::unique_ptr<PID> turnPID;
    std::unique_ptr<PID> anglePID;
    std::unique_ptr<SlewController> driveSlew;
    okapi::QLength angleCorrectionRadius;
};

}