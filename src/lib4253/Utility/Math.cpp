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


double Math::angleWrap360(double angle){
	return angle - 360.0 * std::floor(angle * (1.0 / 360.0));
}

double Math::angleWrap180(double angle){
	return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

double Math::angleWrap90(double angle){
	double newAngle = angleWrap180(angle);
	return angleWrap180(newAngle + (abs(newAngle) > 90) * 180);
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

/*
std::pair<double, double> scaleSpeed(const double& linear, const double& yaw, const double& max) const{
    double left = linear - yaw;
    double right = linear + yaw;
    return desaturate(left, right, max);
}

std::pair<double, double> desaturate(const double& left, const double& right, const double& max) const{
    double leftPower = left, rightPower = right, maxPower = fmax(std::fabs(left), std::fabs(right));
    if(maxPower > std::fabs(max)){
        leftPower = left / maxPower * max;
        rightPower = right / maxPower * max;
    }

    return {leftPower, rightPower};
}   

std::pair<okapi::QSpeed, okapi::QSpeed> inverseKinematics(okapi::QSpeed velocity, okapi::QAngularSpeed angularVelocity) const{
    double angVel = angularVelocity.convert(okapi::radps);
    okapi::QSpeed diff = angVel * dimension.wheelTrack.convert(okapi::meter) * okapi::mps;
    return {velocity - diff, velocity + diff};
}

std::pair<okapi::QAcceleration, okapi::QAcceleration> inverseKinematics(okapi::QAcceleration acceleration, okapi::QAngularAcceleration angularAcceleration) const{
    double angAccel = angularAcceleration.convert(okapi::radps2);
    okapi::QAcceleration diff = angAccel * dimension.wheelTrack.convert(okapi::meter) * okapi::mps2;
    return {acceleration - diff, acceleration + diff};
}
*/

okapi::QLength circumradius(const Translation& left, const Translation& mid, const Translation& right){
        Point A = left;
        Point B = mid;
        Point C = right;

        QLength a = B.distTo(C);
        QLength b = A.distTo(C);
        QLength c = A.distTo(B);
        auto a2 = a * a, b2 = b * b, c2 = c * c;

        Point pa = A * (a2 * (b2 + c2 - a2) / ((b+c)*(b+c)-a2) / (a2-(b-c)*(b-c))).convert(okapi::number);
        Point pb = B * (b2 * (a2 + c2 - b2) / ((a+c)*(a+c)-b2) / (b2-(a-c)*(a-c))).convert(okapi::number);
        Point pc = C * (c2 * (a2 + b2 - c2) / ((a+b)*(a+b)-c2) / (c2-(a-b)*(a-b))).convert(okapi::number);

        Point center = pa + pb + pc;

        QLength radius = center.distTo(A);

		return radius;
}


}