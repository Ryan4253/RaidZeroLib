#include "lib4253/Utility/Math.hpp"
namespace lib4253::Math{

QLength angleToArcLength(QAngle angle, QLength rad){
  return angle.convert(radian) * rad;
}

QAngle arcLengthToAngle(QLength dist, QLength rad){
  return dist / rad * radian;
}

double angleWrap360(double angle){
	return angle - 360.0 * std::floor(angle * (1.0 / 360.0));
}

double angleWrap180(double angle){
	return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

double angleWrap90(double angle){
	double newAngle = angleWrap180(angle);
	return angleWrap180(newAngle + (abs(newAngle) > 90) * 180);
}

QAngle angleWrap360(QAngle angle){
	return angle - 360.0 * floor(angle * (1.0 / 360.0), 1 * radian);
}

QAngle angleWrap180(QAngle angle){
	return angle - 360.0 * floor((angle + 180.0 * degree) * (1.0 / 360.0), 1 * radian);
}

QAngle angleWrap90(QAngle angle){
	QAngle newAngle = angleWrap180(angle);
	return angleWrap180(newAngle + (abs(newAngle) > 90 * degree) * 180 * degree);
}

double velToRPM(QSpeed vel, QLength radius, double gearRatio){
	return (vel.convert(mps) / radius.convert(meter) * 60 / (2*M_PI)) / gearRatio;
}

QSpeed rpmToVel(double rpm, QLength radius, double gearRatio){
	return (rpm * gearRatio) / 60 * 2 * M_PI * radius.convert(meter) * mps;
}

double sinc(double x){
	if(std::abs(x) < 1e-9){
		return 1.0 - 1.0 / 6.0 * x * x;
	} 
	else{
		return std::sin(x) / x;
	}
}

QLength circumradius(const Translation& left, const Translation& mid, const Translation& right){
        Point A = left;
        Point B = mid;
        Point C = right;

        QLength a = B.distTo(C);
        QLength b = A.distTo(C);
        QLength c = A.distTo(B);
        auto a2 = a * a, b2 = b * b, c2 = c * c;

        Point pa = A * (a2 * (b2 + c2 - a2) / ((b+c)*(b+c)-a2) / (a2-(b-c)*(b-c))).convert(number);
        Point pb = B * (b2 * (a2 + c2 - b2) / ((a+c)*(a+c)-b2) / (b2-(a-c)*(a-c))).convert(number);
        Point pc = C * (c2 * (a2 + b2 - c2) / ((a+b)*(a+b)-c2) / (c2-(a-b)*(a-b))).convert(number);

        Point center = pa + pb + pc;

        QLength radius = center.distTo(A);

		return radius;
}

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c){
	double discriminant = b * b - 4 * a * c;
	if(discriminant == 0){
		return std::make_pair(-b / (2 * a), -b / (2 * a));
	}
	else if(discriminant > 0){
		return std::make_pair((-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a));
	}
	
	return std::nullopt;
}




}
