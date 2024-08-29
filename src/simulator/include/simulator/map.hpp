/* @Author: YueLin */

#pragma once

#include <vector>

namespace sim
{
    const int X = 0, Y = 1, Z = 2;

    class Map
    {
        public:
            double resolution;
            int size[X + Y + Z];
            double size0[X + Y + Z];
            std::vector<std::vector<std::vector<bool>>> map;
            std::vector<std::vector<std::vector<bool>>> exp;
        
        public:
            Map(){}
            Map(double, double, double, double);
        
        public:
            void clear();
            void expand(double);
            void random(double, double, double, double, double, int, int);
    };
}
