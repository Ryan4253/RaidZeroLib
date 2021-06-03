#include "main.h"

namespace lib4253{
using namespace okapi;
class Motor : public okapi::Motor{
  private:
    std::shared_ptr<Motor> motor;
    MotorVelocityController velController;
    EmaFilter velFilter;

  public:
    Motor(std::int8_t iport, std::tuple<double, double, double> constant);
    Motor() = default;
    int32_t setVelocity(std::int16_t ivelocity);
    int32_t setVelocty(std::int16_t ivelocity, std::int16_t iacceleration);
};
}