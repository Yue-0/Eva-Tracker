/* @Author: YueLin */

#pragma once

#include <vector>

namespace simulator
{
    const int X = 0, Y = 1, Z = 2;

    class Map
    {
        public:
            bool*** map;
            bool*** exp;
            int size[3];
            double** sdf;
            double size0[3];
            double expansion, resolution;
        
        public:
            ~Map();
            Map() = default;
            Map(double, double, double, double);
        
        public:
            void clear();
            void distance();
            void expand(double);
            void random(double, double, double, 
                        double, double, double, int, int);
    };
}
