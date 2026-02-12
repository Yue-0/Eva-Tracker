/* @Author YueLin */

#include "planner/map.hpp"

namespace eva_tracker
{
    Map::~Map()
    {
        for(int x = 0; x < size.x(); x++)
        {
            for(int y = 0; y < size.y(); y++)
            {
                delete[] data[x][y];
                delete[] flag[x][y];
                delete[] points[x][y];
            }
            delete[] data[x];
            delete[] flag[x];
            delete[] points[x];
        }
        delete[] data;
        delete[] flag;
        delete[] points;
    }

    Map::Map(double l, double w, double h, double r): resolution(r)
    {
        /* Initialize */
        offset << l / 2, w / 2, 0;
        size << std::round(l / r), std::round(w / r), std::round(h / r);

        /* Allocate memory */
        data = new bool**[size.x()];
        flag = new bool**[size.x()];
        points = new pcl::PointXYZ**[size.x()];
        for(int x = 0; x < size.x(); x++)
        {
            data[x] = new bool*[size.y()];
            flag[x] = new bool*[size.y()];
            points[x] = new pcl::PointXYZ*[size.y()];
            for(int y = 0; y < size.y(); y++)
            {
                data[x][y] = new bool[size.z()];
                flag[x][y] = new bool[size.z()];
                points[x][y] = new pcl::PointXYZ[size.z()];
                for(int z = 0; z < size.z(); z++)
                    data[x][y][z] = flag[x][y][z] = false;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> Map::map() const
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                for(int z = 0; z < size.z(); z++)
                    if(flag[x][y][z])
                        cloud.push_back(points[x][y][z]);
        cloud.width = cloud.points.size();
        cloud.is_dense = true;
        cloud.height = 1;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ> Map::update(
        const pcl::PointCloud<pcl::PointXYZ>& cloud, 
        const Eigen::Vector3d& center,
        Eigen::Vector3d* target,
        double expansion,
        double range
    ){
        /* Initialize */;
        double r = 1 / resolution;
        expansion = std::round(expansion * r);

        /* Clear map */
        int x1 = std::max((center.x() + offset.x() - range) * r, 0.);
        int y1 = std::max((center.y() + offset.y() - range) * r, 0.);
        int x2 = std::min((center.x() + offset.x() + range) * r, size.x() - 1.);
        int y2 = std::min((center.y() + offset.y() + range) * r, size.y() - 1.);
        #pragma omp parallel for collapse(2)
        for(int x = x1; x <= x2; x++)
        for(int y = y1; y <= y2; y++)
        {
            std::fill_n(data[x][y], size.z(), false);
            std::fill_n(flag[x][y], size.z(), false);
        }

        /* Filter */
        pcl::PointCloud<pcl::PointXYZ> filtered;
        for(pcl::PointXYZ point: cloud)
        {
            /* Filter the target */
            if(target[1][0] > 1e-3 && 
               std::fabs(point.x - target[0].x()) < target[1][1] &&
               std::fabs(point.y - target[0].y()) < target[1][1] &&
               std::fabs(point.z - target[0].z()) < target[1][2]) continue;

            /* Boundary check */
            int x0 = std::round((point.x + offset.x()) * r);
            int y0 = std::round((point.y + offset.y()) * r);
            int z0 = std::round((point.z + offset.z()) * r);
            if(std::min(std::min(x0, y0), z0) < 0 ||
               x0 >= size.x() || y0 >= size.y() || z0 >= size.z())
                continue;
            
            filtered.push_back(point);
        }

        /* Update */
        for(pcl::PointXYZ point: filtered)
        {
            /* Discretize */
            int x0 = std::round((point.x + offset.x()) * r);
            int y0 = std::round((point.y + offset.y()) * r);
            int z0 = std::round((point.z + offset.z()) * r);
            points[x0][y0][z0] = point;
            flag[x0][y0][z0] = true;

            /* Expand */
            y1 = std::max(y0 - expansion, 0.);
            x1 = std::max(x0 - expansion, 0.);
            int z1 = std::max(z0 - expansion, 0.);
            x2 = std::min(x0 + expansion, size.x() - 1.);
            y2 = std::min(y0 + expansion, size.y() - 1.);
            int z2 = std::min(z0 + expansion, size.z() - 1.);

            /* Fill */
            #pragma omp parallel for collapse(2)
            for(int x = x1; x <= x2; x++)
            for(int y = y1; y <= y2; y++)
                std::fill_n(data[x][y] + z1, z2 - z1 + 1, true);
        }
        
        return filtered;
    }
}
