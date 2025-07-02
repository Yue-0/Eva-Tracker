/* @Author YueLin */

#pragma once

#include <cmath>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace eva_tracker
{
    class Map
    {
        public:
            int size[3];
            double resolution;
            Eigen::Vector3d offset;

        private:
            bool*** data;
            bool*** flag;
            pcl::PointXYZ*** points;
            const int X = 0, Y = 1, Z = 2;

        public:
            ~Map();
            Map(double, double, double, double);
            pcl::PointCloud<pcl::PointXYZ> map();
            pcl::PointCloud<pcl::PointXYZ> update(
                pcl::PointCloud<pcl::PointXYZ>&,
                Eigen::Vector3d*, Eigen::Vector3d, double, double
            );
                
            bool get(Eigen::Vector3d point)
            {
                double r = 1 / resolution;
                int x = std::round((point[X] + offset[X]) * r);
                int y = std::round((point[Y] + offset[Y]) * r);
                int z = std::round((point[Z] + offset[Z]) * r);
                return data[std::max(std::min(x, size[X] - 1), 0)]
                           [std::max(std::min(y, size[Y] - 1), 0)]
                           [std::max(std::min(z, size[Z] - 1), 0)];
            }
    };
}
