/**
 * @file MotorVelocityController.hpp
 * @author Ryan Liao (23ryanl@students.tas.tw) & Jason Zhou (24kasonz@students.tas.tw)
 * @brief Accurate velocity controller for motors
 * @version 1.2
 * @date 2022-02-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include "lib4253/Utility/Units.hpp"


namespace lib4253{
using namespace okapi;

/**
 * @brief Custom feedforward velocity controller class
 * 
 */
class MotorFFController{
    public:
    struct Gains {
        double kVel{0.0}, kAcc{0.0}, kDec{0.0}, kP_Pos{0.0}, kP_Vel{0.0};
    
        /**
         * @brief Construct a new Gains object
         * 
         */
        Gains() = default;

        /**
         * @brief Destroy the Gains object
         * 
         */
        ~Gains() = default;

        /**
         * @brief Construct a new Gains object
         * 
         * @param ikVel velocity constant
         * @param ikAcc acceleration constant
         * @param ikDec deceleration constant
         * @param ikP_Pos proportional constant for position
         * @param ikP_Vel proportional constant for velocity
         */
        Gains(double ikVel, double ikAcc, double ikDec, double ikP_Pos, double ikP_Vel);

        /**
         * @brief checks whether two gains are equal
         * 
         * @param rhs the gain to compare to
         * @return whether the two gains are equal
         */
        bool operator==(const Gains &rhs) const;

        /**
         * @brief checks whether two gains are not equal
         * 
         * @param rhs the gain to compare to
         * @return whether the two gains are not equal
         */
        bool operator!=(const Gains &rhs) const;
    };

    /**
     * @brief Constructs a new FFVelocityController object
     * 
     */
    MotorFFController() = default;

    /**
     * @brief Destroys the FFVelocityController object
     * 
     */
    ~MotorFFController() = default;

    /**
     * @brief Constructs a new FFVelocityController object
     * 
     * @param ikVel velocity constant
     * @param ikAcc acceleration constant
     * @param ikDec deceleration constant
     * @param ikP_Pos proportional constant for position
     * @param ikP_Vel proportional constant for velocity
     */
    MotorFFController(double ikVel, double ikAcc, double ikDec, double ikP_Pos, double ikP_Vel);

    /**
     * @brief Constructs a new FFVelocityController object
     * 
     * @param Gains controller gains
     */
    MotorFFController(const Gains& iGain);

    /**
     * @brief Calculates one step in accordance to the given parameters
     * 
     * @param iPosition desired position, in ft
     * @param iVelocity desired velocity, in ftps
     * @param iAcceleration desired acceleration, in ftps2
     * @param iCurrentPos current position, in ft
     * @param iCurrentVel current velocity, in ftps
     * @return calculated power to reach the target velocity
     */
    double step(double iPosition, double iVelocity, double iAcceleration, double iCurrentPos, double iCurrentVel);

    /**
     * @brief Calculates one step in accordance to the given parameters
     * 
     * @param iPosition desired position
     * @param iVelocity desired velocity
     * @param iAcceleration desired acceleration
     * @param iCurrentPos current position
     * @param iCurrentVel current velocity
     * @return double calculated power to reach the target velocity
     */
    double step(QLength iPosition, QSpeed iVelocity, QAcceleration iAcceleration, QLength iCurrentPos, QSpeed iCurrentVel);

    /**
     * @brief Returns the target power
     * 
     * @return target power
     */
    double getTargetPower() const;

    /**
     * @brief Returns constant
     * 
     * @return constants
     */
    Gains getGains() const;
    
    /**
     * @brief Set the Gains object
     * 
     * @param gains
     * 
     */
    void setGains(const Gains& iGain);


    private:
        Gains gains;
        double power{0};
    };
        
}

