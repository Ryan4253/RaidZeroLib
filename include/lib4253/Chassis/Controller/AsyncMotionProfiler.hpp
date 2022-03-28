#pragma once
#include "lib4253/Utility/TaskWrapper.hpp"
#include "lib4253/Trajectory/Trajectory.hpp"
#include "lib4253/Utility/StateMachine.hpp"
#include "lib4253/Controller/MotorVelocityController.hpp"
#include "lib4253/Controller/LinearMotionProfile.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace lib4253{

/**
 * @brief an enum containing all possible states for our motion profile controller
 * 
 */
enum class MotionProfileState{
    MOVE, FOLLOW, IDLE
};

enum class TurnType{
    GLOBAL, LOCAL
};

// forward declare
template class StateMachine<MotionProfileState>;

/**
 * @brief class that allows us to control our chassis asynchronously using motion profiles
 * 
 */
class AsyncMotionProfiler : public StateMachine<MotionProfileState, MotionProfileState::IDLE>, public TaskWrapper {
    protected:
    /**
     * @brief Construct a new Async Motion Profiler object (using all custom velocity control)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile constraint to generate
     * @param iLeftLinear left velocity controller for linear motion profile
     * @param iRightLinear right velocity controller for linear motion profile
     * @param iLeftTrajectory left velocity controller for trajectories
     * @param iRightTrajectory right velocity controller for trajectories
     * @param iTimeUtil timer utility
     */
    AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                        std::unique_ptr<LinearMotionProfile> iMove, 
                        std::unique_ptr<MotorFFController> iLeftLinear, 
                        std::unique_ptr<MotorFFController> iRightLinear,
                        std::unique_ptr<MotorFFController> iLeftTrajectory,
                        std::unique_ptr<MotorFFController> iRightTrajectory,
                        const okapi::TimeUtil& iTimeUtil);

    /**
     * @brief Construct a new Async Motion Profiler object (using internal velocity control)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile constraint to generate
     * @param iTimeUtil timer utility
     */
    AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                    std::unique_ptr<LinearMotionProfile> iMove, 
                    const okapi::TimeUtil& iTimeUtil);

    /**
     * @brief Construct a new Async Motion Profiler object (using either custom or internal velocity control depending on the constructor)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile to generate
     * @param iLeft left velocity controller (true for linear, false for trajectory)
     * @param iRight right velocity controller (true for linear, false for trajectory)
     * @param velFlag whether linear is custom (true) or trajectory is custom (true)
     * @param iTimeUtil timer utility
     */
    AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                    std::unique_ptr<LinearMotionProfile> iMove, 
                    std::unique_ptr<MotorFFController> iLeft,
                    std::unique_ptr<MotorFFController> iRight,
                    bool velFlag,
                    const okapi::TimeUtil& iTimeUtil);

    void operator=(const AsyncMotionProfiler& rhs) = delete;

    /**
     * @brief generate this class using AsyncMotionProfilerBuilder only
     * 
     */
    friend class AsyncMotionProfilerBuilder;

    public:

    /**
     * @brief Set the target distance to move
     * 
     * @param iDistance target distance
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */
    void setTarget(okapi::QLength iDistance, bool waitUntilSettled = false);

    /**
     * @brief Set the Target trajectoryr
     * 
     * @param iPath target trajectory to move in
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(const Trajectory& iPath, bool waitUntilSettled = false);

     /**
     * @brief Set the Target angle to turn
     * 
     * @param iPath target angle to turn
     * @param iType whether to do a global or local turn
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(okapi::QAngle iAngle, TurnType iType = TurnType::GLOBAL, bool waitUntilSettled = false);

    void logLeftVelocity(bool flag);

    void logRightVelocity(bool flag);

    void logLeftPosition(bool flag);

    void logRightPosition(bool flag);

    /**
     * @brief stop the chassis from moving
     * 
     */
    void stop();

    /**
     * @brief blocks the current movement until the current movement is complete
     * 
     */
    void waitUntilSettled();

    protected:
    std::shared_ptr<okapi::ChassisController> chassis;
    std::shared_ptr<okapi::AbstractMotor> leftMotor;
    std::shared_ptr<okapi::AbstractMotor> rightMotor;

    std::unique_ptr<LinearMotionProfile> profiler;
    std::unique_ptr<MotorFFController> leftLinear{nullptr};
    std::unique_ptr<MotorFFController> rightLinear{nullptr};
    std::unique_ptr<MotorFFController> leftTrajectory{nullptr};
    std::unique_ptr<MotorFFController> rightTrajectory{nullptr};

    okapi::TimeUtil timeUtil;
    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;
    okapi::QTime maxTime{0.0};

    Trajectory path;
    pros::Mutex lock;

    bool trajectoryCustom = false;
    bool linearCustom = false;

    /**
     * @brief task loop
     * 
     */
    void loop() override;
};


/**
 * @brief An AsyncMotionProfile builder class which allows more intuitive instantiation of the class
 * 
 */
class AsyncMotionProfilerBuilder{
    public:
    /**
     * @brief Constructs a new Async Motion Profiler Builder object
     * 
     */
    AsyncMotionProfilerBuilder();

    /**
     * @brief Destroys the Async Motion Profiler Builder object
     * 
     */
    ~AsyncMotionProfilerBuilder() = default;

    /**
     * @brief sets the chassis object for the profiler to output to
     * 
     * @param iChassis the chassis object to output to
     * @return AsyncMotionProfilerBuilder& an ongoing builder
     */
    AsyncMotionProfilerBuilder& withOutput(std::shared_ptr<okapi::ChassisController> iChassis);

    /**
     * @brief sets the motion profile generator to use for linear movements
     * 
     * @param iProfiler the profile generator
     * @return AsyncMotionProfilerBuilder& 
     */
    AsyncMotionProfilerBuilder& withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler);

    /**
     * @brief sets the motor controller to use for linear movements
     * 
     * @param iLeft 
     * @param iRight 
     * @return AsyncMotionProfilerBuilder& 
     */
    AsyncMotionProfilerBuilder& withLinearController(MotorFFController iLeft, MotorFFController iRight);

    /**
     * @brief sets the motor controller to use when following paths    
     * 
     * @param iLeft 
     * @param iRight 
     * @return AsyncMotionProfilerBuilder& 
     */
    AsyncMotionProfilerBuilder& withTrajectoryController(MotorFFController iLeft, MotorFFController iRight);

    /**
     * @brief builds the async motion profiler object with the specified parameters. The thread is started automaically
     * 
     * @return std::shared_ptr<AsyncMotionProfiler> the built async motion profiler
     */
    std::shared_ptr<AsyncMotionProfiler> build();

    private:
    std::unique_ptr<LinearMotionProfile> profile;
    std::shared_ptr<okapi::ChassisController> chassis;
    MotorFFController leftL;
    MotorFFController rightL;
    MotorFFController leftT;
    MotorFFController rightT;

    bool linearInit = false;
    bool trajInit = false;
    bool driveInit = false;
    bool profileInit = false;
};
}
