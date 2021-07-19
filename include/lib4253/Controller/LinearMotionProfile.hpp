#pragma once
#include "lib4253/Utility/Units.hpp"
#include<iostream>
#include<vector>

namespace lib4253{

template<typename Unit>
class LinearMotionProfile{
    protected:
    using Distance = decltype(Unit{1.0});
    using Velocity = decltype(Unit{1.0} / okapi::QTime{1.0});
    using Acceleration = decltype(Velocity{1.0} / okapi::QTime{1.0});
    using Jerk = decltype(Acceleration{1.0} / okapi::QTime{1.0});

    public:
    

    LinearMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk = 2000000000 * Jerk{1.0});
    virtual ~LinearMotionProfile() = default;

    bool isInitialized() const;
    okapi::QTime getTime() const;
    bool isSettled() const;

    virtual void setDistance(const Distance& iTarget) = 0;

    virtual std::pair<Velocity, Acceleration> calculate(const okapi::QTime& currentTime) = 0;
    virtual std::pair<Velocity, Acceleration> calculate(const Distance& currentDist) = 0;

    protected:
    Velocity maxVelocity;
    Acceleration maxAcceleration;
    Jerk maxJerk;

    Distance targetDist;
    okapi::QTime totalTime;
    bool initialize;
    bool reversed;
    bool settled;
};

template<typename Unit>
class TrapezoidalMotionProfile : public LinearMotionProfile<Unit>{
    public:
    using Distance = decltype(Unit{1.0});
    using Velocity = decltype(Unit{1.0} / okapi::QTime{1.0});
    using Acceleration = decltype(Velocity{1.0} / okapi::QTime{1.0});
    using Jerk = decltype(Acceleration{1.0} / okapi::QTime{1.0});

    public:
    TrapezoidalMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk = 2000000000 * Jerk{1.0});
    ~TrapezoidalMotionProfile() = default;

    void setDistance(const Distance& iTarget) override;

    std::pair<Velocity, Acceleration> calculate(const okapi::QTime& currentTime) override;
    std::pair<Velocity, Acceleration> calculate(const Distance& currentDist) override;

    private:
    std::vector<okapi::QTime> timePhase;
    std::vector<Distance> distPhase;
};

template<typename Unit>
class SCurveMotionProfile : public LinearMotionProfile<Unit>{
    using Distance = decltype(Unit{1.0});
    using Velocity = decltype(Unit{1.0} / okapi::QTime{1.0});
    using Acceleration = decltype(Velocity{1.0} / okapi::QTime{1.0});
    using Jerk = decltype(Acceleration{1.0} / okapi::QTime{1.0});

    public:
    SCurveMotionProfile(const Velocity& iMaxVelocity, const Acceleration& iMaxAcceleration, const Jerk& iMaxJerk = 2000000000 * Jerk{1.0});
    ~SCurveMotionProfile() = default;

    void setDistance(const Distance& iTarget) override;

    std::pair<Velocity, Acceleration> calculate(const okapi::QTime& currentTime) override;
    std::pair<Velocity, Acceleration> calculate(const Distance& currentDist) override;

    private:
    std::vector<okapi::QTime> timePhase;
    std::vector<Distance> distPhase;
    std::vector<Velocity> velPhase;
};


}


