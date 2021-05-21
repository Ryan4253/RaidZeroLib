/**
 * @file Slew.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Slew controller
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "main.h"


/**
 * @brief Slew controller class
 *
 */
class SlewController{
  protected:
  int speed;
  int accStep;
  int decStep;

  public:
  /**
   * @brief Construct a new Slew Controller object
   *
   * @param accel acceleration step (how much the velocity increases everytime the robot refreshes)
   * @param decel deceleration step (^ vise versa)
   */
  SlewController(int accel, int decel);

  /**
   * @brief Construct a new Slew Controller object
   *
   */
  SlewController();

  /**
   * @brief Set acceleration and deceleration steps
   *
   * @param a acceleration step
   * @param d deceleration step
   */
  void setStep(double a, double d);

  /**
   * @brief Resets slew controller
   * 
   */
  void reset();

  /**
   * @brief Calculates output power
   *
   * @param target target power
   * @return (possibly) modified power
   */
  double step(double target);
};
