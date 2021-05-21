/**
 * @file Filter.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Filter
 * @version 0.1
 * @date 2021-05-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include "main.h"
<<<<<<< HEAD

/**
 * @brief 4253B custom programming library
 * 
 */
namespace lib4253{

  /**
   * @brief Filter class
   * 
   */
  class Filter{
    public:
    /**
     * @brief Resets filters
     * 
     */
=======

class FilterBase{
  public:
>>>>>>> e28f0a1052c337f3570dcdcd9a98ec1947c8505c
    virtual void reset() = 0;

    /**
     * @brief Converts raw values through filter
     * 
     * @param input raw values
     * @return filtered values
     */
    virtual double filter(double input) = 0;
<<<<<<< HEAD
  };

}
=======
};
>>>>>>> e28f0a1052c337f3570dcdcd9a98ec1947c8505c
