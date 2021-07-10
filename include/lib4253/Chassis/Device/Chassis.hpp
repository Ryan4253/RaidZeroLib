#pragma once
#include "lib4253/Utility/TaskWrapper.hpp"
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Utility/Settler.hpp"
#include "lib4253/Utility/StateMachine.hpp"
#include "lib4253/Utility/Units.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "lib4253/Chassis/Device/Motor.hpp"

#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include <atomic>

namespace lib4253{

enum class DriveState{
    TANK, ARCADE
};

class Chassis: public TaskWrapper, public StateMachine<DriveState>{
    public:	
	// Constructor
	Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const ChassisScales& iScale,
			std::shared_ptr<IMU> imu,
			std::unique_ptr<Slew> iSlew = nullptr,
			std::unique_ptr<PID> iDrivePID = nullptr, 
			std::unique_ptr<PID> iTurnPID = nullptr,
			std::unique_ptr<PID> iAnglePID = nullptr);
    
	Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const ChassisScales& iScale,
			std::unique_ptr<Slew> iSlew = nullptr,
			std::unique_ptr<PID> iDrivePID = nullptr, 
			std::unique_ptr<PID> iTurnPID = nullptr,
			std::unique_ptr<PID> iAnglePID = nullptr,
			std::shared_ptr<IMU> imu = nullptr);

    ~Chassis() = default;
    
	// Task Function
	void loop() override;

	// initializer / setter / getter
	void initialize() const;
	void setBrakeType(const AbstractMotor::brakeMode& iMode) const;
	void resetSensor() const;
	double getIMUReading() const;
	double getEncoderReading() const;
	double getLeftEncoderReading() const;
	double getRightEncoderReading() const;
    ChassisScales getScales() const;

	// drive movement functions
	void setPower(const double& lPower, const double& rPower) const;
    void setPower(const std::pair<double, double>& power) const;
	void setVelocity(const double& lVelocity, const double& rVelocity) const;
    void setVelocity(const std::pair<double, double>& velocity) const;
    void setVelocity(const std::pair<okapi::QSpeed, okapi::QAcceleration>& leftKinematics, const std::pair<okapi::QSpeed, okapi::QAcceleration>& rightKinematics) const;
	void move(const double& lPower, const double& rPower, const QTime& timeLim) const;
	void moveDistance(const okapi::QLength& dist, Settler = Settler::getDefaultSettler()) const;
	void turnAngle(const okapi::QAngle& angle, Settler = Settler::getDefaultSettler()) const;

    // chassis math functions
    std::pair<double, double> desaturate(const double& left, const double& right, const double& max) const;
	std::pair<double, double> scaleSpeed(const double& linear, const double& yaw, const double& max) const;
    std::pair<okapi::QSpeed, okapi::QSpeed> inverseKinematics(okapi::QSpeed velocity, okapi::QAngularSpeed angularVelocity) const;
    std::pair<okapi::QAcceleration, okapi::QAcceleration> inverseKinematics(okapi::QAcceleration acceleration, okapi::QAngularAcceleration angularAcceleration) const;

    // driver control functions
	void tank(const double& left, const double& right);
	void arcade(const double& fwd, const double& yaw);

	private:
	std::vector<std::shared_ptr<Motor> > left {nullptr};
    std::vector<std::shared_ptr<Motor> > right {nullptr};
    std::shared_ptr<IMU> inertial {nullptr};
    ChassisScales dimension;

	std::unique_ptr<Slew> driveSlew {nullptr};
    std::unique_ptr<PID> drivePID {nullptr};
    std::unique_ptr<PID> turnPID {nullptr};
    std::unique_ptr<PID> anglePID {nullptr};

    double lControllerY = 0, rControllerX = 0, rControllerY = 0;
};
}
/*
class Drive{
    public:

    Drive(const std::initializer_list<okapi::Motor> &l, const std::initializer_list<okapi::Motor> &r);
    Drive& withOdometry(const CustomOdometry& tracker);
    Drive& withDimensions(std::tuple<double> wheel, std::tuple<double, double> gear, std::tuple<double> track);
    Drive& withDrivePID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withTurnPID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withPurePursuit(std::tuple<double> lookAhead, std::tuple<double> turnGain, std::tuple<double, double> kinematics);
    Drive& withSlew(int acc, int dec);
    Drive& withVelocityFeedForward(std::tuple<double, double, double> l, std::tuple<double, double, double> r);
    void initialize();

    State getState();
    void setState(State s);

    void moveDistanceLMP(double distance);
    void moveDistanceLMPD(double distance);

    void followPath(SimplePath path);


  protected:

    private:
    std::shared_ptr<CustomOdometry> odom{nullptr};
    PID drivePID; PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;
    LinearMotionProfileController* bruhMobile;


};

*/
