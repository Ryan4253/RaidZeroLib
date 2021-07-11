/**
 * @file EMA.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Exponential moving average (EMA) filter
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
// #include "main.h"
#include "lib4253/Filter/AbstractFilter.hpp"

namespace lib4253{

/**
 * @brief Exponential moving average (EMA) filter class - inherited from Filter
 *
 */
class EmaFilter : public AbstractFilter<double>{
    public:
    /**
     * @brief Construct a new Ema Filter object
     *
     */
    EmaFilter() = default;
    
    /**
     * @brief Construct a new Ema Filter object
     *
     * @param a alpha gain
     */
    EmaFilter(const double& a);

    /**
     * @brief Destroys the Ema Filter object
     * 
     */
    ~EmaFilter() = default;

    /**
     * @brief Sets EMA gain
     *
     * @param a alpha gain
     */
    void setGain(const double& a);

    void initialize() override;

    /**
     * @brief Resets EMA filter
     *
     */
    void reset() override;

    /**
     * @brief Filters raw values
     *
     * @param input raw values
     * @return filtered values
     */
    double filter(const double& input) override;

    /**
     * @brief Gets filtered values
     *
     * @return filtered values
     */
    double getOutput() const override;

    private:
    double alpha{1}, output{0}, prevOutput{0};
    bool run = false;
};
}
