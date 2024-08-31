/* @Author YueLin */

#pragma once

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "utils.hpp"

namespace eva_tracker
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    class Map
    {
        private:
            bool*** data;
        
        public:
            Point<int> size;
            double resolution;

        public:
            ~Map();
            Map(){}
            Map(double, double, double, double);
        
        public:
            void update(PointCloud&, double);
            std::vector<Point<int>> neighbors(Point<int>&, int, int, int);
            std::vector<Point<int>> neighbors(Point<int>& point, int distance)
            {return neighbors(point, distance, distance, distance);}
        
        private:
            void clear();
        
        public:
            bool get(int x, int y, int z)
            {
                return data[std::max(std::min(x, size.x - 1), 0)]
                           [std::max(std::min(y, size.y - 1), 0)]
                           [std::max(std::min(z, size.z - 1), 0)];
            }
    };
}
