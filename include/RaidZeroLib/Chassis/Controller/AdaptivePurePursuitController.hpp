/**
 * @file PurePursuit.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Pure pursuit controller - https://media.istockphoto.com/vectors/kitten-guiding-puppy-with-bone-vector-id1283672337?k=6&m=1283672337&s=612x612&w=0&h=yi3d4m_LMXtYzhZZamDOLE800JJ69VL1TVd1fGbFWSo=
 * @version 1.0
 * @date 2021-07-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Trajectory/Spline/PurePursuitPath.hpp"
#include "lib4253/Chassis/Device/Odometry.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "lib4253/Controller/MotorVelocityController.hpp"
#include "lib4253/Utility/TaskWrapper.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "lib4253/Trajectory/Geometry/Pose.hpp"
#include <optional>
#include <mutex>


namespace lib4253 {
using namespace okapi;

class AsyncAdaptivePurePursuitController : public TaskWrapper{
    public:

    AsyncAdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& iChassis, 
                                  const PurePursuitGains& iGains,
                                  QLength iLookAheadDist,
                                  const MotorFFController& iLeftController,
                                  const MotorFFController& iRightController,
                                  const TimeUtil& iTimeUtil);

    AsyncAdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& iChassis, 
                              	  const PurePursuitGains& iGains,
                              	  QLength iLookAheadDist,
                              	  const TimeUtil& iTimeUtil);

    void operator=(const AsyncAdaptivePurePursuitController& rhs) = delete;

    void followPath(const DiscretePath& iPath);

    void setLookAhead(QLength iLookAhead);

    void setGains(const PurePursuitGains& iGains);

    void waitUntilSettled();

    private:

    /** 
     * @brief Initializes pure pursuit controller
     *
     */
    void initialize();

    std::optional<double> getT(const Point& iStart, const Point& iEnd, const Pose& iPos);

    /**
     * @brief Calculates closest point on path to generate desired motor velocities
     *
     */
    int getClosestPoint(const Pose& currentPos);

    // TO DO: optimize    

    /**
      * @brief Calculates the look ahead point (aka. the next target point)
      *
      */
    Point getLookAheadPoint(const Pose& currentPos);

    /**
     * @brief Calculates curvature of the path to assist with turning
     *
     */
    QCurvature calcCurvature(const Pose& iPos, const Point& lookAheadPt);

    /**
      * @brief Calculates power for drive
      *
      */
    std::pair<QSpeed, QSpeed> calcVelocity(QCurvature iCurvature, int iClosestPt);

    std::pair<QAcceleration, QAcceleration> calcAcceleration(QCurvature iCurvature, int iClosestPt);

    /**
     * @brief Checks if drive has stopped
     * 
     * @return true - drive stopped
     * @return false - drive in motion
     */
    bool isSettled();


	void loop() override;


    std::shared_ptr<OdomChassisController> chassis;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;
    ChassisScales scales;
    
    QLength lookAhead;
	TimeUtil timeUtil;
	std::optional<MotorFFController> leftController;
	std::optional<MotorFFController> rightController;
	PurePursuitGains gains;

	std::unique_ptr<AbstractTimer> timer;
	std::unique_ptr<AbstractRate> rate;

    PurePursuitPath path;
    bool isReversed;
	pros::Mutex lock;

	std::optional<int> prevClosest {std::nullopt};
	int prevLookAheadIndex {0};
	double prevLookAheadT{0};

    bool settled{false};

};
}
