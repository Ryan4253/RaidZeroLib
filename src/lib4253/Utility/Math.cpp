#include "lib4253/Utility/Math.hpp"
namespace lib4253{

okapi::QLength Math::angleToArcLength(const okapi::QAngle& angle, const okapi::QLength& rad){
  return angle.convert(okapi::radian) * rad;
}

okapi::QAngle Math::arcLengthToAngle(const okapi::QLength& dist, const okapi::QLength& rad){
  return dist / rad * okapi::radian;
}

okapi::QAngle Math::angleToYaw(const okapi::QAngle& angle, const okapi::ChassisScales& scale){
  okapi::QLength arc = Math::angleToArcLength(angle, scale.wheelDiameter / 2);
  return Math::arcLengthToAngle(arc, scale.wheelTrack);
}

double Math::cubicControl(double power){
  return power * power * power / 16129;
}

okapi::QAngle Math::angleWrap360(const okapi::QAngle& angle){
  return angle - 360.0 * floor(angle * (1.0 / 360.0), 1 * okapi::radian);
}

okapi::QAngle Math::angleWrap180(const okapi::QAngle& angle){
  return angle - 360.0 * floor((angle + 180.0 * okapi::degree) * (1.0 / 360.0), 1 * okapi::radian);
}

okapi::QAngle Math::angleWrap90(const okapi::QAngle& angle){
  okapi::QAngle newAngle = angleWrap180(angle);
  return angleWrap180(newAngle + (abs(newAngle) > 90 * okapi::degree) * 180 * okapi::degree);
}

double Math::linearVelToRPM(double linVel, double gearRatio, double radius){
  return (linVel / radius * 60 / (2*M_PI)) / gearRatio;
}

double Math::RPMToLinearVel(double rpm, double gearRatio, double radius){
  return (rpm * gearRatio) / 60 * 2 * M_PI * radius;
}

double Math::sinc(double x){
  if(std::abs(x) < 1e-9){
    return 1.0 - 1.0 / 6.0 * x * x;
  } 
  else{
    return std::sin(x) / x;
  }
}

double Math::clamp(double val, double min, double max){
  if(min > max){
    throw std::runtime_error("Math::Clamp: minimun is larger than maximum!");
  }
  return fmin(max, fmax(min, val));
}

constexpr long long Math::power(unsigned val, unsigned exp){
  if(exp == 1 || exp == 0){
    return 1;
  }
  else if(exp % 2 == 0){
    long long pow = power(val, exp / 2);
    return pow * pow;
  }
  else{
    long long pow = power(val, exp / 2);
    return pow * pow * val;
  }
}

constexpr long long Math::fact(unsigned n){
  return n * fact(n-1);
}

constexpr long long Math::ncr(unsigned n, unsigned r){
    if (r == 0) return 1;

    /*
     Extra computation saving for large R,
     using property:
     N choose R = N choose (N-R)
    */
    if (r > n / 2) return ncr(n, n - r); 

    long long res = 1; 

    for (int i = 1; i <= r; i++)
    {
        res *= n - i + 1;
        res /= i;
    }

    return res;
}
}