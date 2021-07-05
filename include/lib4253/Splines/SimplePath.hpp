/**
 * @file SimplePath.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Simple Path
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <vector>
#include "lib4253/Splines/Geometry/Point2D.hpp"

namespace lib4253{

    /**
     * @brief Simple path structure
     *
     */
    struct SimplePath{
        private:
        std::vector<Point2D> rawPoint;
        std::vector<Point2D> waypoint;
        std::vector<double> distance;
        std::vector<double> curvature;
        double spacing = 0.5;

        /**
         * @brief Smooths given path
         *
         * @param a
         * @param b
         * @param tolerance
         */
        void smoothPath(double a, double b, double tolerance);

        /**
         * @brief Smooths given path with qunitic spline
         *
         */
        void smoothPathQuinticSpline();
        void populatePath();
        void updateDistance();
        void updateCurvature();

        public:
        SimplePath();
        SimplePath(std::vector<Point2D> v);
        SimplePath(Point2D v);
        void injectPoint(Point2D v);
        void injectPoint(std::vector<Point2D> v);
        void generatePath(double a, double b, double tolerance);
        void clearPath();
        Point2D getWaypoint(int index);
        double getDistance(int index);
        double getCurvature(int index);
        int getSize();
    };

    template<typename T>
    class SimplePath2{
        public:
        SimplePath2(std::initializer_list<T> _waypoint);
        SimplePath2<T>& generate(int step, bool includeLast = true){
            for(int i = 0; i < step; i++){
                waypoint[i].x;
            }
        }
        SimplePath2<T>& smooth(double a, double b, double tolerance);

        std::vector<T> waypoint;
    };

    //SimplePath2<Point2D> line = SimplePath2<Point2D>({{3, 3}, {3, 6}}).generate(50).smooth(1, 1, 1);
    //SimplePath2<Pose2D> line2 = SimplePath2<Pose2D>({{3, 3, 0}, {3, 6, 90}}).generate(50).smooth(1, 1, 1);


}