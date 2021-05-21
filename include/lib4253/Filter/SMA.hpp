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
#include "main.h"

/**
 * @brief 4253B custom programming library
 * 
 */
namespace lib4253{

  /**
   * @brief Simple moving average filter (SMA) - inherited from Filter
   * 
   */
  class SmaFilter : public Filter{
    std::queue<double> value; // ngl i was thinking about coding a segment tree
    int maxSize;
    double total, output;

    /**
     * @brief Construct a new Sma Filter object
     * 
     */
    SmaFilter();

    /**
     * @brief Construct a new Sma Filter object
     * 
     * @param size maximum size
     */
    SmaFilter(int size);

    /**
     * @brief filters raw values through SMA filter
     * 
     * @param input raw values
     * @return filtered values
     */
    double filter(double input);

    /**
     * @brief Gets filtered values
     * 
     * @return filtered value
     */
    double getOutput();

    /**
     * @brief Sets max size for filter
     * 
     * @param size maximum size
     */
    void setMaxSize(int size);

    /**
     * @brief Resets SMA filter
     * 
     */
    void reset();
  };

}
