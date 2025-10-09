#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "au/au.hpp"

namespace rz {

au::QuantityD<au::Radians> ParametricPath::getTheta(double t) const noexcept {
    return getVelocity(t).theta();
}

au::QuantityD<au::Inverse<au::Meters>> ParametricPath::getCurvature(double t) const noexcept {
    const Point velocity = getVelocity(t);
    const Point acceleration = getAcceleration(t);
    const auto speed = velocity.mag();

    if (speed == au::ZERO) {
        return au::ZERO;
    }

    return velocity.wedge(acceleration) / (speed * speed * speed);
}

double ParametricPath::stepT(double t, au::QuantityD<au::Meters> distance) const noexcept {
    if(t == 1 || distance == au::ZERO){
        return 1;
    }

    double dt = std::min((distance / getVelocity(t).mag()), 1.0 - t);
    double maxDt = dt;
    for (int i = 0; i < 8 && t + maxDt < 1.0 && getLength(t, t + maxDt) < distance; ++i) {
        maxDt = std::min(maxDt * 2, 1.0 - t);
    }

    constexpr unsigned int ITERS = 5;
    constexpr auto ABS_TOL = au::meters(1e-5);
    for (unsigned int it = 0; it < ITERS; ++it) {
        const auto s = getLength(t, t + dt);
        const auto err = s - distance;
        if (abs(err) <= ABS_TOL){
            break;
        }

        const auto sp = getVelocity(t + dt).mag();
        const double dtNew = dt - err / sp;

        if (dtNew <= 0.0){
            dt = 0.5 * dt;
        }   
        else if (dtNew >= maxDt){
            dt = 0.5 * (dt + maxDt);
        }
        else{
            dt = dtNew;
        }              
    }

    return std::min(1.0, t + dt);
}

au::QuantityD<au::Meters> ParametricPath::getLength(double tStart, double tEnd) const noexcept {
    if (tStart == tEnd) {
        return au::ZERO;
    }

    using Meters = au::QuantityD<au::Meters>;

    const auto gaussLegendre5 = [&](double start, double end){
        constexpr std::array<double, 5> X = 
            {-0.9061798459386640,-0.5384693101056831,0.0,0.5384693101056831,0.9061798459386640};
        constexpr std::array<double, 5> W = 
            {0.2369268850561891, 0.4786286704993665,0.5688888888888889,0.4786286704993665,0.2369268850561891};

        const double c = 0.5*(start + end);
        const double h = 0.5*(end - start);
        Meters s = au::ZERO;

        for (std::size_t i = 0; i < 5; ++i){
            s += W[i] * getVelocity(c + h*X[i]).mag();
        }

        return h * s;
    };

    const auto gaussLegendre8 = [&](double start, double end){
        constexpr std::array<double, 8> X = 
            {-0.9602898564975363,-0.7966664774136267,-0.5255324099163290,-0.1834346424956498,
            0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363};
        constexpr std::array<double, 8> W = 
            {0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
            0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};

        const double c = 0.5*(start + end);
        const double h = 0.5*(end - start);
        Meters s = au::ZERO;

        for (std::size_t i = 0; i < 8; ++i){
            s += W[i] * getVelocity(c + h*X[i]).mag();
        }

        return h * s;        
    };

    std::function<Meters(double, double, Meters, unsigned int)> gaussLegendreAdaptive;
    gaussLegendreAdaptive = [&](double start, double end, au::QuantityD<au::Meters> tol, unsigned int depth){
        const Meters length5 = gaussLegendre5(start, end);
        const Meters length8 = gaussLegendre8(start, end);

        if (abs(length8 - length5) <= tol || depth == 0) {
            return length8;
        }

        const double m = (start + end) / 2;
        return gaussLegendreAdaptive(start, m, tol / 2, depth-1) + gaussLegendreAdaptive(m, end, tol / 2, depth-1);
    };

    constexpr Meters TOLERANCE = au::meters(0.001);
    constexpr unsigned int MAX_DEPTH = 10;
    return gaussLegendreAdaptive(tStart, tEnd, TOLERANCE, MAX_DEPTH);
}

DiscretePath ParametricPath::toDiscrete(au::QuantityD<au::Meters> distance, bool includeEnd) const {
    std::vector<Point> path;

    for(double t = 0; t < 1;){
        path.push_back(getPoint(t));
        t = stepT(t, distance);
    }

    const auto end = getPoint(1);
    if(includeEnd && path.back().distTo(end) >= distance / 2){
        path.push_back(getPoint(1));
    }

    return DiscretePath(path);
}

} // namespace rz
