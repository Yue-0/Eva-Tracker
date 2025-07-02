/* @Author YueLin */

#include "planner/map.hpp"
#include "iostream"

namespace eva_tracker
{
    Map::~Map()
    {
        for(int x = 0; x < size[X]; x++)
        {
            for(int y = 0; y < size[Y]; y++)
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

    Map::Map(double l, double w, double h, double r)
    {
        /* Initialize */
        resolution = r;
        r = 1 / resolution;
        size[X] = std::round(l * r);
        size[Y] = std::round(w * r);
        size[Z] = std::round(h * r);
        offset[X] = l * 0.5;
        offset[Y] = w * 0.5;
        offset[Z] = 0.0;

        /* Allocate memory */
        data = new bool**[size[X]];
        flag = new bool**[size[X]];
        points = new pcl::PointXYZ**[size[X]];
        for(int x = 0; x < size[X]; x++)
        {
            data[x] = new bool*[size[Y]];
            flag[x] = new bool*[size[Y]];
            points[x] = new pcl::PointXYZ*[size[Y]];
            for(int y = 0; y < size[Y]; y++)
            {
                data[x][y] = new bool[size[Z]];
                flag[x][y] = new bool[size[Z]];
                points[x][y] = new pcl::PointXYZ[size[Z]];
                for(int z = 0; z < size[Z]; z++)
                    data[x][y][z] = flag[x][y][z] = false;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> Map::map()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                for(int z = 5; z < size[Z]; z++)
                    if(flag[x][y][z])
                        cloud.push_back(points[x][y][z]);
        cloud.width = cloud.points.size();
        cloud.is_dense = true;
        cloud.height = 1;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ> Map::update(
        pcl::PointCloud<pcl::PointXYZ>& cloud, 
        Eigen::Vector3d* target,
        Eigen::Vector3d center,
        double expansion,
        double range
    ){
        /* Initialize */;
        double r = 1 / resolution;
        expansion = std::round(expansion * r);

        /* Clear map */
        int x1 = std::max((center[X] + offset[X] - range) * r, 0.);
        int y1 = std::max((center[Y] + offset[Y] - range) * r, 0.);
        int x2 = std::min((center[X] + offset[X] + range) * r, size[X] - 1.);
        int y2 = std::min((center[Y] + offset[Y] + range) * r, size[Y] - 1.);
        for(int x = x1; x <= x2; x++)
            for(int y = y1; y <= y2; y++)
                for(int z = 0; z < size[Z]; z++)
                    data[x][y][z] = flag[x][y][z] = false;

        /* Filter */
        pcl::PointCloud<pcl::PointXYZ> filtered;
        for(pcl::PointXYZ point: cloud)
        {
            /* Filter the target */
            if(target[1][0] > 1e-3 && 
               std::fabs(point.x - target[0][0]) < target[1][1] &&
               std::fabs(point.y - target[0][1]) < target[1][1] &&
               std::fabs(point.z - target[0][2]) < target[1][2]) continue;

            /* Boundary check */
            int x0 = std::round((point.x + offset[X]) * r);
            int y0 = std::round((point.y + offset[Y]) * r);
            int z0 = std::round((point.z + offset[Z]) * r);
            if(std::min(std::min(x0, y0), z0) < 0 ||
               x0 >= size[X] || y0 >= size[Y] || z0 >= size[Z])
                continue;
            
            filtered.push_back(point);
        }

        /* Update */
        for(pcl::PointXYZ point: filtered)
        {
            /* Discretize */
            int x0 = std::round((point.x + offset[X]) * r);
            int y0 = std::round((point.y + offset[Y]) * r);
            int z0 = std::round((point.z + offset[Z]) * r);
            points[x0][y0][z0] = point;
            flag[x0][y0][z0] = true;

            /* Expand */
            y1 = std::max(y0 - expansion, 0.);
            x1 = std::max(x0 - expansion, 0.);
            int z1 = std::max(z0 - expansion, 0.);
            x2 = std::min(x0 + expansion, size[X] - 1.);
            y2 = std::min(y0 + expansion, size[Y] - 1.);
            int z2 = std::min(z0 + expansion, size[Z] - 1.);

            /* Fill */
            for(int x = x1; x <= x2; x++)
                for(int y = y1; y <= y2; y++)
                    for(int z = z1; z <= z2; z++)
                        data[x][y][z] = true;
        }
        
        return filtered;
    }
}
