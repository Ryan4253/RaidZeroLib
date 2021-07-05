/**
 * @file VelocityController.hpp
 * @author Jason Zhou (24JasonZ@students.tas.tw)
 * @brief Base class for all velocity controllers
 * @version 0.1
 * @date 2021-07-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

/**
 * @brief Abstract velocity controller class
 * 
 * @param T gain type
 */
template<typename T> 
class AbstractVelocityController {
    public:
    /**
     * @brief Sets gain - pure virtual
     * 
     * @param gain controller gain
     */
    virtual void setGain(const T& gain) = 0;
    
    /**
     * @brief Resets the controller - pure virtual
     * 
     */
    virtual void reset() = 0;

    /**
     * @brief Initializes the controller - pure virtual
     * 
     */
    virtual void initialize() = 0;

    /**
     * @brief Calculates output power - pure virtual
     * 
     * @param val input value
     * @return output value
     */
    virtual double step(const double& val) = 0;
};