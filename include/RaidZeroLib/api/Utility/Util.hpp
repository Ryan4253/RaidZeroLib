#pragma once
#include "okapi/api/units/RQuantity.hpp"
#include <type_traits>

namespace rz {
using namespace okapi;

template <typename... Args>
void RQuantityChecker(RQuantity<Args...>) {
}

template <typename T>
concept isRQuantity = requires(T t) {
    RQuantityChecker(std::declval<T>());
    { t.getValue() } -> std::convertible_to<double>;
};

} // namespace rz