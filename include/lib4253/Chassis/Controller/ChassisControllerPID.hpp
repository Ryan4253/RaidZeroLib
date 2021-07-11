#include "lib4253/Chassis/Device/Chassis.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
namespace lib4253{

class ChassisControllerPID{
    ChassisControllerPID(
        std::shared_ptr<Chassis> iChassis,
        std::unique_ptr<PID> iDrivePID,
        std::unique_ptr<PID> iTurnPID,
        std::unique_ptr<PID> iAnglePID,
        std::unique_ptr<Slew> iSlew
    );

    ~ChassisControllerPID() = default;

    void moveDistance(const okapi::QLength& dist, Settler = Settler::getDefaultSettler()) const;
	void turnAngle(const okapi::QAngle& angle, Settler = Settler::getDefaultSettler()) const;

    private:
    std::shared_ptr<Chassis> chassis;
    std::unique_ptr<Slew> slew {nullptr};
    std::unique_ptr<PID> drivePID {nullptr};
    std::unique_ptr<PID> turnPID {nullptr};
    std::unique_ptr<PID> anglePID {nullptr};
};


}