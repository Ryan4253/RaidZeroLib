#pragma once
#include "lib4253/Chassis/Device/Odometry.hpp"
#include "lib4253/Utility/TaskWrapper.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "display/lvgl.h"

// credits to Theo Lemay from team 7842F for coding this display
// original repo: https://github.com/theol0403/7842F-Competition-Code
namespace lib4253{
class OdomDisplay : public TaskWrapper{
  public:
    lv_obj_t* container = nullptr;
    Odometry* tracker = nullptr;

    lv_obj_t* field = nullptr;
    double fieldDim = 0;

    OdomDisplay(lv_obj_t*, Odometry*);
    OdomDisplay(lv_obj_t*, lv_color_t, Odometry*);
    ~OdomDisplay();

    static lv_res_t tileAction(lv_obj_t*);
    static lv_res_t resetAction(lv_obj_t*);
    static lv_res_t btnmAction(lv_obj_t*);

    void loop() override;
};
}