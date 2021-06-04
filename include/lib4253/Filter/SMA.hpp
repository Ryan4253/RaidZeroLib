/**
 * @file SMA.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Simple moving average filter (SMA) (y r there so many filters wtf)
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Filter/Filter.hpp"
#include<queue>
namespace lib4253{


  /**
   * @brief Simple moving average filter (SMA) - inherited from Filter
   *
   */
template<int N> 
class SmaFilter : public FilterBase{
    std::queue<double> value; // ngl i was thinking about coding a segment tree
    double total, output;

    /**
     * @brief Construct a new Sma Filter object
     *
     */
	SmaFilter() = default;

    /**
     * @brief filters raw values through SMA filter
     *
     * @param input raw values
     * @return filtered values
     */
    double filter(double input) override;

    /**
     * @brief Gets filtered values
     *
     * @return filtered value
     */
    double getOutput();

    /**
     * @brief Resets SMA filter
     *
     */
    void reset() override;
};
}
