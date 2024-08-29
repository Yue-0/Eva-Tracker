/* @Author YueLin */

#pragma once

#include <Eigen/Eigen>

#include "utils.hpp"

namespace eva_tracker
{
    typedef Point<float> Velocity;
    typedef Point<float> Acceleration;

    class BSpline
    {
        private:
            double dt;
            Path control;
            int degree, n;
            std::vector<double> knots;

        public:
            BSpline(const unsigned int d = 3): degree(d){}
        
        public:
            /* Get the trajectory of points on the B-spline curve */
            Path trajectory();

            /* Get the point on the B-spline curve at time */
            Point<double> operator[](double time);

            /* Constructing a B-spline curve from a path */
            void create(Path&, Velocity&, Acceleration&, const double);

            /* Get the derivative of a B-spline curve */
            BSpline derivative(const unsigned int);
            BSpline derivative(){return derivative(1);}

            /* Update B-spline curve */
            void update(Path&, const double);

            /* Get all control points */
            Eigen::VectorXd controls();

            /* Get degree */
            int deg() {return degree;}
        
        private:
            void update();
    };
}
