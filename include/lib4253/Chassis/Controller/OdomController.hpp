#include "lib4253/Chassis/Device/Odometry.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "lib4253/Trajectory/Geometry/Pose.hpp"

namespace lib4253{
using namespace okapi;

class OdomController{
    public:
    OdomController(std::shared_ptr<OdomChassisController> iChassis, 
                                   QLength iAngleCorrectionRadius,
                                   std::unique_ptr<IterativePosPIDController> iDrivePID, 
                                   std::unique_ptr<IterativePosPIDController> iTurnPID, 
                                   std::unique_ptr<IterativePosPIDController> iHeadingPID, 
                                   std::unique_ptr<SlewController> iSlew);
                                
    ~OdomController() = default;  

    void moveToPoint(const Point& target, double turnScale);
    void moveToX(QLength targetX);
    void moveToY(QLength targetY);
    void turnToAngle(QAngle angle);
    void turnToPoint(const Point& target);

    private:
    std::shared_ptr<OdomChassisController> chassis;
    std::unique_ptr<IterativePosPIDController> drivePID;
    std::unique_ptr<IterativePosPIDController> turnPID;
    std::unique_ptr<IterativePosPIDController> headingPID;
    std::unique_ptr<SlewController> driveSlew;
    QLength angleCorrectionRadius;
};

}