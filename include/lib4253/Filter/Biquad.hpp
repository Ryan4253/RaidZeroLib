/**
 * @file Biquad.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Biquadratic filter
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Filter/Filter.hpp"
#include <math.h>
namespace lib4253{

  /**
   * @brief Biquadratic filter class: Inherited class from Filter
   *
   */
class BiquadFilter : public FilterBase{
    double prevInput[2];
    double prevOutput[2];
    double a1, a2, b0, b1, b2; // a0 defaulted to 0
    double sample, cutoff, initVal;

  /**
   * @brief Enumerator for High & Low pass
   *
   */
  enum state{
    HIGHPASS, LOWPASS
  };

  /**
   * @brief Construct a new Biquad Filter object
   *
   * @param type type of filter
   * @param sampleFreq sample frequency
   * @param cutoffFreq cut off frequency
   * @param initValue intial value
   */
  BiquadFilter(BiquadFilter::state type, double sampleFreq, double cutoffFreq, double initValue);

  /**
   * @brief Resets the filter
   *
   */
  void reset();

    /**
     * @brief Runs raw input through the Biquadratic filter
     *
     * @param input raw values to be converted
     * @return filted values
     */
    double filter(double input);
};
}
