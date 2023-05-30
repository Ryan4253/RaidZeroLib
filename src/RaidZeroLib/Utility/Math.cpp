#include "RaidZeroLib/Utility/Math.hpp"

namespace rz{

double constrainAngle360(double iAngle){
	return iAngle - 360.0 * std::floor(iAngle * (1.0 / 360.0));
}

double constrainAngle180(double iAngle){
	return iAngle - 360.0 * std::floor((iAngle + 180.0) * (1.0 / 360.0));
}

QAngle constrainAngle360(QAngle iAngle){
	return iAngle - 360.0 * floor(iAngle * (1.0 / 360.0), 1 * radian);
}

QAngle constrainAngle180(QAngle iAngle){
	return iAngle - 360.0 * floor((iAngle + 180.0 * degree) * (1.0 / 360.0), 1 * radian);
}

double sinc(double x){
	if(std::abs(x) < 1e-9){
		return 1.0 - 1.0 / 6.0 * x * x;
	} 
	else{
		return std::sin(x) / x;
	}
}

QLength circumradius(const Translation& iLeft, const Translation& iMid, const Translation& iRight){
        Point A = iLeft;
        Point B = iMid;
        Point C = iRight;

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
