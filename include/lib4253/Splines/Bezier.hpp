#include "Point2D.hpp"
namespace lib4253{

int fact(int n) {
    if (n == 0 || n == 1)
        return 1;
    else
        return n * fact(n - 1);
}

int comb(int n, int r){
    return fact(n) / fact(r) / fact(n-r);
}

int power(int val, int deg){
    int sol = 1;
    for(int i = 0; i < deg; i++){
        sol *= val;
    }

    return sol;
}

class Bezier{
    private:
    std::vector<Point2D> control_point;
    Point2D calc(double t){
        Point2D res;
        int degree = control_point.size()-1;
        for(int i = 0; i < control_point.size(); i++){
            res += control_point[i] * (comb(degree, i) * power(1-t, degree-i) * power(t, i));
        }
        return res;
    }

    public:
    std::vector<Point2D> generate(std::initializer_list<Point2D> control, int step){
        control_point = control;
        std::vector<Point2D> spline;
        for(int i = 0; i < step; i++){
            spline.push_back(calc(1.0 * i / (step-1)));
        }
        return spline;
    }
};
}
