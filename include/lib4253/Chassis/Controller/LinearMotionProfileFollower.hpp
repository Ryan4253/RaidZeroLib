#include "lib4253/Controller/LinearMotionProfile.hpp"
#include "lib4253/Chassis/Device/Chassis.hpp"
#include "lib4253/Chassis/Device/Odometry.hpp"
namespace lib4253{

class LinearMotionProfileFollower{
    public:
    LinearMotionProfileFollower(const std::shared_ptr<Chassis>& iChassis, 
                                const std::shared_ptr<LinearMotionProfile<okapi::QLength>>& iDriveProfiler, 
                                const std::shared_ptr<LinearMotionProfile<okapi::QAngle>>& iTurnProfiler);
    void moveDistance(const okapi::QLength& dist);
    void turnAngle(const okapi::QAngle& dist);

    private:
    std::shared_ptr<Chassis> chassis;
    std::shared_ptr<LinearMotionProfile<okapi::QLength>> driveProfiler;
    std::shared_ptr<LinearMotionProfile<okapi::QAngle>> turnProfiler;
    okapi::TimeUtil timer = std::move(okapi::TimeUtilFactory::createDefault());;
};

}