/* @Author YueLin */

#pragma once

#include "utils.hpp"

namespace eva_tracker
{
    class ESDF
    {
        private:
            double maximum;
            double*** esdf;
            Point<int> size;
            double resolution;
            Point<double> offset;
        
        public:
            ESDF(){}
            ~ESDF();

            /* Construct the ESDF */
            ESDF(Point<double>, double, double);   // RC-ESDF
            ESDF(double, double, double, double);  // FOV-ESDF

            /* Get the ESDF value */
            double get(Point<double> point) {return value(transform(point));}

            /* Get the gradient value of ESDF */
            Point<double> gradient(Point<double>);

            /* Get the index of the maximum value */
            Point<double> argmax();

            /* Get the maximum value */
            double max();

        private:
            void build(bool***);
            double value(Point<double>);
            Point<double> transform(Point<double>&);
            void dt(std::vector<double>&, std::vector<double>&);
            void dt(std::vector<double>&);
    };
}
