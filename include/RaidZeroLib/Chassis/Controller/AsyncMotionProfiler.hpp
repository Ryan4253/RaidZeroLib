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
    AsyncMotionProfiler(std::shared_ptr<ChassisController> iChassis, 
                        std::unique_ptr<LinearMotionProfile> iMove, 
                        const std::optional<MotorFFController>& iLeftLinear, 
                        const std::optional<MotorFFController>& iRightLinear,
                        const std::optional<MotorFFController>& iLeftTrajectory,
                        const std::optional<MotorFFController>& iRightTrajectory,
                        const TimeUtil& iTimeUtil);

    void operator=(const AsyncMotionProfiler& rhs) = delete;

    /**
     * @brief generate this class using AsyncMotionProfilerBuilder only
     * 
     */
    friend class Builder;

    public:

    /**
     * @brief Set the target distance to move
     * 
     * @param iDistance target distance
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */
    void setTarget(QLength iDistance, bool waitUntilSettled = false);

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
    void setTarget(QAngle iAngle, QAngle iCurrentAngle = 0_deg, bool waitUntilSettled = false);

    void setConstraint(ProfileConstraint iConstraint);

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

    /**
     * @brief An AsyncMotionProfile builder class which allows more intuitive instantiation of the class
     * 
     */
    class Builder{
        public:
        /**
         * @brief Constructs a new Async Motion Profiler Builder object
         * 
         */
        Builder();

        /**
         * @brief Destroys the Async Motion Profiler Builder object
         * 
         */
        ~Builder() = default;

        /**
         * @brief sets the chassis object for the profiler to output to
         * 
         * @param iChassis the chassis object to output to
         * @return AsyncMotionProfilerBuilder& an ongoing builder
         */
        Builder& withOutput(std::shared_ptr<ChassisController> iChassis);

        /**
         * @brief sets the motion profile generator to use for linear movements
         * 
         * @param iProfiler the profile generator
         * @return AsyncMotionProfilerBuilder& 
         */
        Builder& withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler);

        /**
         * @brief sets the motor controller to use for linear movements
         * 
         * @param iLeft 
         * @param iRight 
         * @return AsyncMotionProfilerBuilder& 
         */
        Builder& withLinearController(MotorFFController iLeft, MotorFFController iRight);

        /**
         * @brief sets the motor controller to use when following paths    
         * 
         * @param iLeft 
         * @param iRight 
         * @return AsyncMotionProfilerBuilder& 
         */
        Builder& withTrajectoryController(MotorFFController iLeft, MotorFFController iRight);

        /**
         * @brief builds the async motion profiler object with the specified parameters. The thread is started automaically
         * 
         * @return std::shared_ptr<AsyncMotionProfiler> the built async motion profiler
         */
        std::shared_ptr<AsyncMotionProfiler> build();

        private:
        std::unique_ptr<LinearMotionProfile> profile{nullptr};
        std::shared_ptr<ChassisController> chassis{nullptr};
        std::optional<MotorFFController> leftL{std::nullopt};
        std::optional<MotorFFController> rightL{std::nullopt};
        std::optional<MotorFFController> leftT{std::nullopt};
        std::optional<MotorFFController> rightT{std::nullopt};

    };

    protected:
    std::shared_ptr<ChassisController> chassis;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;

    std::unique_ptr<LinearMotionProfile> profiler;
    std::optional<MotorFFController> leftLinear{std::nullopt};
    std::optional<MotorFFController> rightLinear{std::nullopt};
    std::optional<MotorFFController> leftTrajectory{std::nullopt};
    std::optional<MotorFFController> rightTrajectory{std::nullopt};

    TimeUtil timeUtil;
    std::unique_ptr<AbstractRate> rate;
    std::unique_ptr<AbstractTimer> timer;
    QTime maxTime{0.0};

    Trajectory path;
    pros::Mutex lock;

    /**
     * @brief task loop
     * 
     */
    void loop() override;


};



}
