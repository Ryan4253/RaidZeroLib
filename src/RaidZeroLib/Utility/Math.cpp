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

std::pair<QSpeed, QSpeed> curvatureToWheelVelocity(QSpeed velocity, QCurvature curvature, QLength wheelTrack, bool isReversed){
	const QSpeed left = velocity * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
	const QSpeed right = velocity * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;

	if(isReversed){
		return {right, left};
	}

	return {left, right};
}

std::pair<QAcceleration, QAcceleration> curvatureToWheelAcceleration(QAcceleration accel, QCurvature curvature, QLength wheelTrack, bool isReversed){
	const QAcceleration left = accel * (2 + curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;
	const QAcceleration right = accel * (2 - curvature.convert(radpm) * wheelTrack.convert(meter)) / 2;

	if(isReversed){
		return {right, left};
	}

	return {left, right};
}

}
