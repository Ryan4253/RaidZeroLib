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
#include "lib4253/Filter/AbstractFilter.hpp"
// #include <math.h>
#define _USE_MATH_DEFINES
#include <cmath>

namespace lib4253{

  /**
   * @brief Biquadratic filter class: Inherited class from Filter
   *
   */
class BiquadFilter : public AbstractFilter<double>{
    public:
    /**
     * @brief Enumerator for High & Low pass
     *
     */
    enum State{
      HIGHPASS, LOWPASS
    };

    BiquadFilter() = default;

    /**
     * @brief Construct a new Biquad Filter object
     *
     * @param type type of filter
     * @param sampleFreq sample frequency
     * @param cutoffFreq cut off frequency
     * @param initValue intial value
     */
    BiquadFilter(const BiquadFilter::State& type, const double& sampleFreq, const double& cutoffFreq, const double& initValue);

    ~BiquadFilter() = default;

    void setGain(const BiquadFilter::State& type, const double& sampleFreq, const double& cutoffFreq, const double& initValue);

    void initialize() override;

    /**
     * @brief Resets the filter
     *
     */
    void reset() override;

    /**
     * @brief Runs raw input through the Biquadratic filter
     *
     * @param input raw values to be converted
     * @return filted values
     */
    double filter(const double& input) override;

    double getOutput() const override;

    private:
    double prevInput[2];
    double prevOutput[2];
    double a1, a2, b0, b1, b2; // a0 defaulted to 0
    double sample, cutoff, initVal;
    double output;
};
}
