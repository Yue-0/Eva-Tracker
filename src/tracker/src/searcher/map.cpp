/* @Author YueLin */

#include "searcher/map.hpp"

namespace eva_tracker
{
    Map::~Map()
    {
        for(int x = 0; x < size.x; x++)
        {
            for(int y = 0; y < size.y; y++)
                delete[] data[x][y];
            delete[] data[x];
        }
        delete[] data;
    }

    Map::Map(double l, double w, double h, double r)
    {
        /* Initialize */
        resolution = r;
        r = 1 / resolution;
        size.x = std::round(l * r);
        size.y = std::round(w * r);
        size.z = std::round(h * r);

        /* Allocate memory */
        data = new bool**[size.x];
        for(int x = 0; x < size.x; x++)
        {
            data[x] = new bool*[size.y];
            for(int y = 0; y < size.y; y++)
                data[x][y] = new bool[size.z];
        }
        clear();
    }

    void Map::update(PointCloud& cloud, double expansion)
    {
        clear();
        double r = 1 / resolution;
        for(pcl::PointXYZ p: cloud)
        {
            Point<int> point(
                std::round(p.x * r + size.x / 2),
                std::round(p.y * r + size.y / 2),
                std::round(p.z * r + size.z / 2)
            );
            for(Point<int> expand: neighbors(point, std::round(expansion * r)))
                data[expand.x][expand.y][expand.z] = true;
        }
    }

    std::vector<Point<int>> Map::neighbors(Point<int>& point, int distance)
    {
        /* Boundary */ 
        std::vector<Point<int>> points; int
        x1 = std::max(point.x - distance, 0),
        y1 = std::max(point.y - distance, 0),
        z1 = std::max(point.z - distance, 0),
        x2 = std::min(point.x + distance, size.x - 1),
        y2 = std::min(point.y + distance, size.y - 1),
        z2 = std::min(point.z + distance, size.z - 1);
        
        /* Add neighbors */
        for(int x = x1; x <= x2; x++)
            for(int y = y1; y <= y2; y++)
                for(int z = z1; z <= z2; z++)
                    if(!data[x][y][z])
                        points.push_back(Point<int>(x, y, z));
        return points;
    }

    void Map::clear()
    {
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    data[x][y][z] = false;
    }
}
