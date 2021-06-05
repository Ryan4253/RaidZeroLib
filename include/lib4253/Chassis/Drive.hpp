#pragma once
#include "lib4253/Utility/TaskWrapper.hpp"
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Chassis/Motor.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include <atomic>

namespace lib4253{
class Chassis: public TaskWrapper{
	public:
    enum class State{
      	TANK = 0, ARCADE = 1	
    };
	
	// Constructor
	Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const ChassisScales& iScale,
			std::shared_ptr<IMU> imu,
			std::unique_ptr<SlewController> _driveSlew = nullptr,
			std::unique_ptr<PID> _drivePID = nullptr, 
			std::unique_ptr<PID> _turnPID = nullptr,
			std::unique_ptr<PID> _anglePID = nullptr);
    
	Chassis(const std::initializer_list<std::shared_ptr<Motor> >& iLeft, 
			const std::initializer_list<std::shared_ptr<Motor> >& iRight, 
			const ChassisScales& iScale,
			std::unique_ptr<SlewController> _driveSlew = nullptr,
			std::unique_ptr<PID> _drivePID = nullptr, 
			std::unique_ptr<PID> _turnPID = nullptr,
			std::unique_ptr<PID> _anglePID = nullptr,
			std::shared_ptr<IMU> imu = nullptr);
    
	// State Machine Functions
	void loop() override;
	State getState() const;
    void setState(const State& s);

	// initializer / setter / getter
	void initialize();
	void setBrakeType(const AbstractMotor::brakeMode& iMode);
	void resetSensor();
	double getIMUReading() const;
	double getEncoderReading() const;
	double getLeftEncoderReading() const;
	double getRightEncoderReading() const;

	// drive movement functions
	void setPower(const double& lPower, const double& rPower);
    void setPower(const std::pair<double, double> power);
	void setVelocity(const double& lVelocity, const double& rVelocity);
    void setVelocity(const std::pair<double, double> velocity);
	void move(const double& lPower, const double& rPower, const QTime& timeLim);
	void moveDistance(const double& dist, const QTime& timeLim);
	void turnAngle(const double& angle, const QTime& timeLim);


	private:
	std::vector<std::shared_ptr<Motor> > left {nullptr};
    std::vector<std::shared_ptr<Motor> > right {nullptr};
    std::shared_ptr<IMU> inertial {nullptr};
    ChassisScales scale;

	std::unique_ptr<SlewController> driveSlew {nullptr};
    std::unique_ptr<PID> drivePID {nullptr};
    std::unique_ptr<PID> turnPID {nullptr};
    std::unique_ptr<PID> anglePID {nullptr};

	std::atomic<State> currentState{State::TANK};

    std::pair<double, double> desaturate(const double& left, const double& right, const double& max);
	std::pair<double, double> scaleSpeed(const double& linear, const double& yaw, const double& max);

	// driver control functions
	void tank(const double& left, const double& right);
	void arcade(const double& fwd, const double& yaw);
};
}
/*
class Drive{
    public:
    enum State{
      TANK, ARCADE
    };

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

    void resetIMU();
    double getAngle();

    void move(double voltage);
    void move(double  voltage, okapi::QTime time);

    void moveDistance(double distance, okapi::QTime timeLimit);
    void moveTo(Point2D target, double turnScale, okapi::QTime timeLimit);
    void turnAngle(double angle, okapi::QTime timeLimit);
    void turnToAngle(double angle, okapi::QTime timeLimit);

    void moveDistanceLMP(double distance);
    void moveDistanceLMPD(double distance);

    void followPath(SimplePath path);
    void followPath(std::string name);

    static void driveTask(void *ptr);

  protected:
    okapi::MotorGroup left{nullptr};
    okapi::MotorGroup right;

    private:
    std::shared_ptr<CustomOdometry> odom{nullptr};
    PID drivePID; PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;
    LinearMotionProfileController* bruhMobile;
    MotorVelocityController leftVelController, rightVelController;

    double gearRatio, trackWidth, wheelSize;
    State driveState = TANK;
    int prevAState = 0;

    Point2D scaleSpeed(double linear, double turn, double turnScale);
    void updateState();
    void run();

    void tank();
    void arcade();
};
}
*/