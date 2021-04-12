#pragma once
#include "main.h"

// credits to Theo Lemay from team 7842F for coding this display
// original repo: https://github.com/theol0403/7842F-Competition-Code


namespace okapi{
  constexpr QLength tile = 2 * foot;
  constexpr QLength court = 12 * foot;
  inline namespace literals{
    constexpr QLength operator"" _tl(long double x) {
      return static_cast<double>(x) * tile;
    }
    constexpr QLength operator"" _crt(long double x) {
      return static_cast<double>(x) * court;
    }
    constexpr QLength operator"" _tl(unsigned long long int x) {
      return static_cast<double>(x) * tile;
    }
    constexpr QLength operator"" _crt(unsigned long long int x) {
      return static_cast<double>(x) * court;
    }
  }
}

class OdomDisplay {
  public:
    lv_obj_t* container = nullptr;
    CustomOdometry* tracker = nullptr;

    lv_obj_t* field = nullptr;
    double fieldDim = 0;

    pros::Task task;

    OdomDisplay(lv_obj_t*, CustomOdometry*);
    OdomDisplay(lv_obj_t*, lv_color_t, CustomOdometry*);
    ~OdomDisplay();

    static lv_res_t tileAction(lv_obj_t*);
    static lv_res_t resetAction(lv_obj_t*);
    static lv_res_t btnmAction(lv_obj_t*);

    void run();
    static void taskFnc(void*);
};
