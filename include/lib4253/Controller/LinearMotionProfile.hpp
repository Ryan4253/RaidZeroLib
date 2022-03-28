#pragma once
#include "lib4253/Utility/Units.hpp"
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Trajectory/Trajectory.hpp"

namespace lib4253{

/**
 * @brief A struct that stores the physical constraints of the chassis
 * 
 */
struct ProfileConstraint{
    QSpeed maxVelocity{0.0};
    QAcceleration maxAcceleration{0.0};
    QAcceleration maxDeceleration{0.0};
    QJerk maxJerk{0.0};

    /**
     * @brief Constructs a new Profile Constraint object
     * 
     * @param maxVel max velocity
     * @param maxAccel max acceleration
     * @param maxDecel max deceleration (only used in trapezoidal motion profiles)
     * @param maxJerk max jerk (only used in s curve motion profiles)
     */
    ProfileConstraint(QSpeed maxVel, QAcceleration maxAccel, QAcceleration maxDecel, QJerk maxJerk);

    ProfileConstraint(QSpeed maxVel, QAcceleration maxAccel, QJerk maxJerk);

    /**
     * @brief Destroys the Profile Constraint object
     * 
     */
    ~ProfileConstraint() = default;

    protected:
    /**
     * @brief Constructs a Profile Constraint object - only accessible by the LinearMotionProfile class
     * 
     */
    ProfileConstraint() = default;

    /**
     * @brief forward declared to be friend
     * 
     */
    friend class LinearMotionProfile;
};

/**
 * @brief An abstract class which acts as a base of all linear motion profile classes
 * 
 */
class LinearMotionProfile {
    protected:
    ProfileConstraint constraint;
    QLength distance{0.0};
    bool isReversed{false};

    std::vector<QTime> timePhase;
    std::vector<QLength> distPhase;
    std::vector<QSpeed> velPhase;
    std::vector<QAcceleration> accPhase;
    std::vector<QJerk> jerkPhase;

    public:
    /**
     * @brief Constructs a new Linear Motion Profile object
     * 
     */
    LinearMotionProfile() = default;

    /**
     * @brief Destroys the Linear Motion Profile object
     * 
     */
    ~LinearMotionProfile() = default;

    /**
     * @brief sets the target distance 
     * 
     * @param iDistance new target distance
     */
    virtual void setDistance(QLength iDistance) = 0;

    /**
     * @brief sets chassis constraints
     * 
     * @param iConstraint new constraint
     */
    virtual void setConstraint(ProfileConstraint iConstraint) = 0;

    /**
     * @brief Gets the total time to run the profile
     * 
     * @return QTime total time to run the profile
     */
    virtual QTime getTotalTime() const = 0;

    /**
     * @brief Gets the target position at a given time
     * 
     * @param time time step to query
     * @return QLength the distance travelled at the target time
     */
    virtual QLength getPosition(QTime time) const = 0;

    /**
     * @brief Gets the target velocity at a given time
     * 
     * @param time time step to query
     * @return QSpeed target velocity at the target time
     */
    virtual QSpeed getVelocity(QTime time) const = 0;

    /**
     * @brief Gets the target acceleration at a given time
     * 
     * @param time time step to query
     * @return QAcceleration target acceleration at the target time
     */
    virtual QAcceleration getAcceleration(QTime time) const = 0;
    
    /**
     * @brief gets the kinematics data (position, velocity, acceleration) at a given time
     * 
     * @param time time step to query
     * @return TrajectoryPoint target kinematics data at the target time
     */
    virtual TrajectoryPoint get(QTime time) const = 0;
};

/**
 * @brief class which generates "trapezoidal linear motion profiles" for the chassis to follow.
 *        Given a target distance, it generates a set of velocities which makes the chassis
 *        accelerate at max acceleration, cruise at max velocity, then decelerate at max
 *        deceleration. As a result, the velocity curve of the chassis' translation looks like
 *        a trapezoid. 
 * 
 *       Since all the methods are inherited from the LinearMotionProfile class, repeated comments
 *       are omitted. 
 */
class TrapezoidalMotionProfile : public LinearMotionProfile{
    private:
    QLength min3Stage = 0 * meter;

    public:
    TrapezoidalMotionProfile(ProfileConstraint iConstraint);
    TrapezoidalMotionProfile() = default;
    ~TrapezoidalMotionProfile() = default;

    void setDistance(QLength iDistance) override;
    void setConstraint(ProfileConstraint iConstraint) override;

    QTime getTotalTime() const override;
    QLength getPosition(QTime time) const override;
    QSpeed getVelocity(QTime time) const override;
    QAcceleration getAcceleration(QTime time) const override;
    TrajectoryPoint get(QTime time) const override;
};

/**
 * @brief class which generates "s curve linear motion profiles" for the chassis to follow
 *        Given a target distance, it generates a set of velocities which makes the chassis
 *        move at max jerk, max acceleration, max velocity, and decelerate the same way. 
 *        As a result, the velocity curve of the chassis' translation looks like
 *        a smooth s shaped trapezoid. 
 * 
 *        Since all the methods are inherited from the LinearMotionProfile class, repeated comments
 *        are omitted. 
 */
class SCurveMotionProfile : public LinearMotionProfile{
    QLength fullDist = 0 * meter;
    QLength minDist = 0 * meter;
    bool fullAccel{true};

    public:
    SCurveMotionProfile(ProfileConstraint iConstraint);
    SCurveMotionProfile() = default;
    ~SCurveMotionProfile() = default;

    void setDistance(QLength iDistance) override;
    void setConstraint(ProfileConstraint iConstraint) override;

    QTime getTotalTime() const override;
    QLength getPosition(QTime time) const override;
    QSpeed getVelocity(QTime time) const override;
    QAcceleration getAcceleration(QTime time) const override;
    TrajectoryPoint get(QTime time) const override;

    private:
    void genFourStage(QLength iDistance);
    void genFiveStage(QLength iDistance);
    void genSixStage(QLength iDistance);
    void genSevenStage(QLength iDistance);
};

}