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
#include "lib4253/Filter/Filter.hpp"
namespace lib4253{

/**
 * @brief Exponential moving average (EMA) filter class - inherited from Filter
 *
 */
class EmaFilter : public FilterBase{
    private:
    double alpha, output = 0, prevOutput = 0;
    bool run = false;
    public:
    /**
     * @brief Construct a new Ema Filter object
     *
     */
    EmaFilter();
    
    /**
     * @brief Construct a new Ema Filter object
     *
     * @param a alpha gain
     */
    EmaFilter(double a);

    /**
     * @brief Sets EMA gain
     *
     * @param a alpha gain
     */
    void setGain(double a);

    /**
     * @brief Gets filtered values
     *
     * @return filtered values
     */
    double getOutput();

    /**
     * @brief Filters raw values
     *
     * @param input raw values
     * @return filtered values
     */
    double filter(double input);

    /**
     * @brief Resets EMA filter
     *
     */
    void reset() override;
};
}
