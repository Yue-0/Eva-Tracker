/* @Author YueLin */

#pragma once

#include <cmath>

#include "Eigen/Eigen"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"

namespace eva_tracker
{
    class Map
    {
        public:
            Eigen::Vector3i size;
            Eigen::Vector3d offset;
            const double resolution;

        private:
            bool*** data;
            bool*** flag;
            pcl::PointXYZ*** points;

        public:
            ~Map();
            Map(double l, double w, double h, double r);
            pcl::PointCloud<pcl::PointXYZ> map() const;
            pcl::PointCloud<pcl::PointXYZ> update(
                const pcl::PointCloud<pcl::PointXYZ>& cloud,
                const Eigen::Vector3d& center, 
                Eigen::Vector3d* target, 
                double expansion, 
                double range
            );
                
            bool get(Eigen::Vector3d point) const
            {
                double r = 1 / resolution;
                int x = std::round((point.x() + offset.x()) * r);
                int y = std::round((point.y() + offset.y()) * r);
                int z = std::round((point.z() + offset.z()) * r);
                return data[std::max(std::min(x, size.x() - 1), 0)]
                           [std::max(std::min(y, size.y() - 1), 0)]
                           [std::max(std::min(z, size.z() - 1), 0)];
            }
    };
}
