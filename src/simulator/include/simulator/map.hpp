/* @Author: YueLin */

#pragma once

#include <vector>

#include "Eigen/Eigen"

namespace simulator
{
    // const int X = 0, Y = 1, Z = 2;

    class Map
    {
        public:
            bool*** map;
            bool*** exp;
            double** sdf;
            Eigen::Vector3i size;
            Eigen::Vector3d size0;
            double expansion, resolution;
        
        public:
            ~Map();
            Map() = default;
            Map(double x, double y, double z, double r);
        
        public:
            void clear();
            void distance();
            void expand(double sz);
            void random(double x1, double y1, double x2, double y2, 
                        double sz, double wh, int seed, int obstacles);
    };
}
