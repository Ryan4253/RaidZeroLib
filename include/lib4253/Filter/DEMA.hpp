/**
 * @file DEMA.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Double exponential moving average (DEMA) filter (literally copied from okapi but okay)
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Filter/AbstractFilter.hpp"

namespace lib4253{

  /**
   * @brief Double exponential moving average (DEMA) filter class - inherited from Filter
   *
   */

class DemaFilter: public AbstractFilter<double>{
    public:
    /**
     * @brief Construct a new Dema Filter object
     *
     */
    DemaFilter() = default;

    /**
     * @brief Construct a new Dema Filter object
     *
     * @param a alpha gain
     * @param b beta gain
     */
    DemaFilter(const double& a, const double& b);

    /**
     * @brief Destroys the Dema Filter object
     * 
     */
    ~DemaFilter() = default;

    /**
     * @brief Set the Gain object
     *
     * @param a alpha gain
     * @param b beta gain
     */
    void setGain(const double& a, const double& b);

    void initialize() override;

    /**
     * @brief Resets the filter
     *
     */
    void reset() override;

    /**
     * @brief Runs raw input through the DEMA filter
     *
     * @param input raw values to be converted
     * @return filtered values
     */
    double filter(const double& input) override;

    /**
     * @brief Gets output values
     *
     * @return filtered values
     */
    double getOutput() const override;

    private:
    double alpha, beta;
    double outputS, outputB;
    double prevOutputS, prevOutputB;
};
}
