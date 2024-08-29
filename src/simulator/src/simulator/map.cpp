/* @Author: YueLin */

#include <cmath>
#include <cstdlib>

#include "simulator/map.hpp"

namespace sim
{
    Map::Map(double x, double y, double z, double r)
    {
        resolution = r; r = 1 / r;
        size[X] = (size0[X] = x) * r;
        size[Y] = (size0[Y] = y) * r;
        size[Z] = (size0[Z] = z) * r;
        size0[X] -= resolution;
        size0[Y] -= resolution;
        size0[Z] -= resolution;
        clear();
    }

    void Map::clear()
    {
        map = std::vector<std::vector<std::vector<bool>>>(
            size[X], std::vector<std::vector<bool>>(
                size[Y], std::vector<bool>(size[Z], false)
            )
        );
        exp = std::vector<std::vector<std::vector<bool>>>(
            size[X], std::vector<std::vector<bool>>(
                size[Y], std::vector<bool>(size[Z], false)
            )
        );
    }

    void Map::expand(double sz)
    {
        int dist = std::round(sz / resolution);
        for(int x0 = 0; x0 < size[X]; x0++)
            for(int y0 = 0; y0 < size[Y]; y0++)
                for(int z0 = 0; z0 < size[Z]; z0++)
                    if(map[x0][y0][z0])
                    {
                        int x1 = std::max(x0 - dist, 0);
                        int y1 = std::max(y0 - dist, 0);
                        int z1 = std::max(z0 - dist, 0);
                        int x2 = std::min(x0 + dist, size[X] - 1);
                        int y2 = std::min(y0 + dist, size[Y] - 1);
                        int z2 = std::min(z0 + dist, size[Z] - 1);
                        for(int x = x1; x <= x2; x++)
                            for(int y = y1; y <= y2; y++)
                                for(int z = z1; z <= z2; z++)
                                    exp[x][y][z] = true;
                    }
    }

    void Map::random(double x1, double y1,
                     double x2, double y2, 
                     double sz, int seed, int obstacles)
    {
        std::srand(seed);
        double r = 1 / resolution;
        double positions[2][2] = {
            {std::round(x1 * r), std::round(y1 * r)},
            {std::round(x2 * r), std::round(y2 * r)}
        };

        /* Generate map */
        int width = size[X] >> 3, height = size[Y] >> 3;
        for(int b = 1; b >= 0; b--)
            for(int obstacle = 0; obstacle < obstacles; obstacle++)
            {
                x1 = rand() % size[X];
                y1 = rand() % size[Y];
                int w = rand() % width + 1;
                int h = rand() % height + 1;
                x2 = std::min(size[X] - 1., x1 + w + 1);
                y2 = std::min(size[Y] - 1., y1 + h + 1);
                int z0 = std::max(std::min(
                    (rand() % size[Z]) << b, size[Z]
                ), (size[Z] >> 1) * b);
                for(int x = x1; x <= x2; x++)
                    for(int y = y1; y <= y2; y++)
                        for(int z = 0; z < z0; z++)
                            map[x][y][z] = b;
            }
        
        /* Obstacles cannot cover robots */
        int clear = sz * r * 2;
        for(int robot = 0; robot < 2; robot++)
        {
            x1 = std::max(positions[robot][0] - clear, 0.);
            y1 = std::max(positions[robot][1] - clear, 0.);
            x2 = std::min(positions[robot][0] + clear, size[X] * 1.);
            y2 = std::min(positions[robot][1] + clear, size[Y] * 1.);
            for(int x = x1; x < x2; x++)
                for(int y = y1; y < y2; y++)
                    for(int z = 0; z < size[Z]; z++)
                        map[x][y][z] = false;
        }
        
        /* Wall */
        // for(int y = 0; y < size[Y]; y++)
        //     for(int z = 0; z < size[Z]; z++)
        //         map[0][y][z] = true;
        // for(int y = 0; y < size[Y]; y++)
        //     for(int z = 0; z < size[Z]; z++)
        //         map[size[X] - 1][y][z] = true;
        // for(int x = 0; x < size[X]; x++)
        //     for(int z = 0; z < size[Z]; z++)
        //         map[x][0][z] = map[x][size[Y] - 1][z] = true;
    }
}
