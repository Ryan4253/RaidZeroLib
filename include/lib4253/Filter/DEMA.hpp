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

#include "main.h"


  /**
   * @brief Double exponential moving average (DEMA) filter class - inherited from Filter
   *
   */

class DemaFilter: public FilterBase{
  private:
    double alpha, beta;
    double outputS, outputB;
    double prevOutputS, prevOutputB;

    public:
    /**
     * @brief Construct a new Dema Filter object
     *
     */
    DemaFilter();

    /**
     * @brief Construct a new Dema Filter object
     *
     * @param a alpha gain
     * @param b beta gain
     */
    DemaFilter(double a, double b);

    /**
     * @brief Set the Gain object
     *
     * @param a alpha gain
     * @param b beta gain
     */
    void setGain(double a, double b);

    /**
     * @brief Gets output values
     *
     * @return filtered values
     */
    double getOutput();

    /**
     * @brief Runs raw input through the DEMA filter
     *
     * @param input raw values to be converted
     * @return filtered values
     */
    double filter(double input);

    /**
     * @brief Resets the filter
     *
     */
    void reset();
  };
