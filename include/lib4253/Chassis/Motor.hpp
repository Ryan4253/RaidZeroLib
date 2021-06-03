#include "main.h"

namespace lib4253{
using namespace okapi;
class Motor : public okapi::Motor{
  private:
    std::unique_ptr<MotorVelocityController> velController {nullptr};
    std::unique_ptr<EmaFilter> velFilter {nullptr};

  public:
    Motor(const int& iport, const okapi::AbstractMotor::gearset& cartridge, const double& ia, const std::tuple<double, double, double>& constant);
    int32_t setRPM(const std::int16_t& ivelocity);
    int32_t setRPM(const std::int16_t& ivelocity, const std::int16_t& iacceleration);
    double getFilteredVelocity();
};
}