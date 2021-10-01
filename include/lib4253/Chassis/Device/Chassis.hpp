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
	// Constructor & Destructor
	Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const okapi::ChassisScales& iScale,
			std::shared_ptr<okapi::IMU> imu = nullptr);

    ~Chassis() = default;
    
	// Task Function
	void loop() override;

	// Initializer
	void initialize() const;
	void resetSensor() const;

    // Getter
	okapi::QAngle getAngle() const;
    okapi::QLength getDistance() const;
	double getEncoderReading() const;
	double getLeftEncoderReading() const;
	double getRightEncoderReading() const;
    okapi::ChassisScales getDimension() const;

	// Setter
    void setBrakeType(const okapi::AbstractMotor::brakeMode& iMode) const;
	void setPower(const double& lPower, const double& rPower) const;
    void setPower(const std::pair<double, double>& power) const;
	void setVelocity(const double& lVelocity, const double& rVelocity) const;
    void setVelocity(const std::pair<double, double>& velocity) const;
    void setVelocity(const std::pair<okapi::QSpeed, okapi::QAcceleration>& leftKinematics, const std::pair<okapi::QSpeed, okapi::QAcceleration>& rightKinematics) const;

    // Chassis Movement Method
	void move(const double& lPower, const double& rPower, const okapi::QTime& timeLim) const;

    // Chassis Math Methods
    std::pair<double, double> desaturate(const double& left, const double& right, const double& max) const;
	std::pair<double, double> scaleSpeed(const double& linear, const double& yaw, const double& max) const;
    std::pair<okapi::QSpeed, okapi::QSpeed> inverseKinematics(okapi::QSpeed velocity, okapi::QAngularSpeed angularVelocity) const;
    std::pair<okapi::QAcceleration, okapi::QAcceleration> inverseKinematics(okapi::QAcceleration acceleration, okapi::QAngularAcceleration angularAcceleration) const;

    // driver control functions
	void tank(const double& left, const double& right);
	void arcade(const double& fwd, const double& yaw);

    /**
     * @brief Although similar to arcade drive, curvature drives allows more accurate
     *        turning by taking the drive's curvature into account instead of just 
     *        controller input. In simpler terms, turning is nerfed. 
     * 
     * @param throttle forwards & backwards, controlled by leftY
     * @param turn turn, controller by rightX
     * @param quickTurn switches between arcade & curvature drive
     */
    void curvature(const double& throttle, const double& turn, const bool& quickTurn);

	private:
	std::vector<std::shared_ptr<Motor> > left {nullptr};
    std::vector<std::shared_ptr<Motor> > right {nullptr};
    std::shared_ptr<okapi::IMU> inertial {nullptr};
    okapi::ChassisScales dimension;

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
    PurePursuitFollower PPTenshi;
    LinearMotionProfileController* bruhMobile;


};

*/
