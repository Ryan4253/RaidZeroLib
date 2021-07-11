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
//#include "main.h"
namespace lib4253{
/**
* @brief Abstract Filter class
*
*/
template<typename T>
class AbstractFilter{
    public:
    virtual void initialize() = 0;
    virtual void reset() = 0;
    virtual T filter(const T& input)  = 0;
    virtual T getOutput() const = 0;
};
}
