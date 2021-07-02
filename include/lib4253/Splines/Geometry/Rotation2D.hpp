#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib4253{

class Rotation{
    public:
    constexpr Rotation() = default;

    Rotation(okapi::QAngle val);

    Rotation(okapi::QLength x, okapi::QLength y);


    private:
    okapi::QAngle value = 0_rad;

};

}