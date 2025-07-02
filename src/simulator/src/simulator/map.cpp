/* @Author: YueLin */

#include <cmath>
#include <limits>
#include <cstdlib>

#include "simulator/map.hpp"

namespace simulator
{
    Map::~Map()
    {
        for(int x = 0; x < size[X]; x++)
        {
            for(int y = 0; y < size[Y]; y++)
            {
                delete[] map[x][y];
                delete[] exp[x][y];
            }
            delete[] sdf[x];
            delete[] map[x];
            delete[] exp[x];
        }
        delete[] sdf;
        delete[] map;
        delete[] exp;
    }

    Map::Map(double x, double y, double z, double r)
    {
        /* Initalize */
        resolution = r; r = 1 / r;
        size[X] = (size0[X] = x) * r;
        size[Y] = (size0[Y] = y) * r;
        size[Z] = (size0[Z] = z) * r;
        size0[X] -= resolution;
        size0[Y] -= resolution;
        size0[Z] -= resolution;

        /* Apply for memory */
        map = new bool**[size[X]];
        exp = new bool**[size[X]];
        sdf = new double*[size[X]];
        for(int x = 0; x < size[X]; x++)
        {
            map[x] = new bool*[size[Y]];
            exp[x] = new bool*[size[Y]];
            sdf[x] = new double[size[Y]];
            for(int y = 0; y < size[Y]; y++)
            {
                map[x][y] = new bool[size[Z]];
                exp[x][y] = new bool[size[Z]];
            }
        }

        /* Initialize map */
        clear();
    }

    void Map::clear()
    {
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
            {
                sdf[x][y] = 0;
                for(int z = 0; z < size[Z]; z++)
                    map[x][y][z] = exp[x][y][z] = false;
            }
    }

    void Map::distance()
    {
        /* Initialize */
        const double INF = std::numeric_limits<double>::infinity();
        std::vector<double> distance, f;
        std::vector<std::vector<double>> negative(
            size[X], std::vector<double>(size[Y], 0)
        );
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                sdf[x][y] = 0;
        
        /* Calculate the distance in the y direction */
        distance.resize(size[Y]);
        for(int x = 0; x < size[X]; x++)
        {
            for(int y = 0; y < size[Y]; y++)
                distance[y] = exp[x][y][0]? 0: INF;
            for(int y = 1; y < size[Y]; y++)
                distance[y] = std::min(distance[y], distance[y - 1] + 1);
            for(int y = size[Y] - 2; y >= 0; y--)
                distance[y] = std::min(distance[y], distance[y + 1] + 1);
            for(int y = 0; y < size[Y]; y++)
                distance[y] *= distance[y];
            for(int y = 0; y < size[Y]; y++)
                sdf[x][y] = distance[y];
        }
        for(int x = 0; x < size[X]; x++)
        {
            for(int y = 1; y < size[Y]; y++)
                distance[y] = exp[x][y][0]? INF: 0;
            distance[0] = distance[size[Y] - 1] = 0;
            for(int y = 1; y < size[Y]; y++)
                distance[y] = std::min(distance[y], distance[y - 1] + 1);
            for(int y = size[Y] - 2; y >= 0; y--)
                distance[y] = std::min(distance[y], distance[y + 1] + 1);
            for(int y = 0; y < size[Y]; y++)
                distance[y] *= distance[y];
            for(int y = 0; y < size[Y]; y++)
                negative[x][y] = distance[y];
        }

        /* Calculate the distance in the y direction */
        f.resize(size[X]);
        distance.resize(size[X]);
        int* v = new int[size[X]];
        double* z = new double[size[X] + 1];
        for(int k, q, y = 0; y < size[Y]; y++)
        {
            double s;
            v[0] = 0; z[0] = -(z[1] = INF);
            for(int x = 0; x < size[X]; x++)
                f[x] = sdf[x][y];
            for(k = q = 1; q < size[X]; q++)
            {
                do {k--;}
                while(z[k] >= (s = (
                    (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
                ) / (2 * (q - v[k]))));
                v[++k] = q;
                z[k++] = s;
                z[k] = INF;
            }
            for(k = q = 0; q < size[X]; q++)
            {
                while(z[++k] < q);
                --k; distance[q] = f[v[k]] + (q - v[k]) * (q - v[k]);
            }
            for(int x = 0; x < size[X]; x++)
                sdf[x][y] = distance[x];
        }
        for(int k, q, y = 0; y < size[Y]; y++)
        {
            double s;
            v[0] = 0; z[0] = -(z[1] = INF);
            for(int x = 0; x < size[X]; x++)
                f[x] = negative[x][y];
            for(k = q = 1; q < size[X]; q++)
            {
                do {k--;}
                while(z[k] >= (s = (
                    (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
                ) / (2 * (q - v[k]))));
                v[++k] = q;
                z[k++] = s;
                z[k] = INF;
            }
            for(k = q = 0; q < size[X]; q++)
            {
                while(z[++k] < q);
                --k; distance[q] = f[v[k]] + (q - v[k]) * (q - v[k]);
            }
            for(int x = 0; x < size[X]; x++)
                negative[x][y] = distance[x];
        }
        delete[] z;
        delete[] v;

        /* Map the result to Euclidean distance */
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                sdf[x][y] = ! exp[x][y][0]
                            ? resolution * std::sqrt(sdf[x][y])
                            : resolution * -std::sqrt(negative[x][y]);
    }

    void Map::expand(double sz)
    {
        expansion = sz;
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
        for(int x = 0; x < size[X]; x++)
            exp[x][0][0] = exp[size[X] - 1][0][0] = true;
        for(int y = 0; y < size[Y]; y++)
            exp[0][y][0] = exp[0][size[Y] - 1][0] = true;
    }

    void Map::random(double x1, double y1,
                     double x2, double y2, 
                     double sz, double wh,
                     int seed, int obstacles)
    {
        std::srand(seed);
        double r = 1 / resolution;
        double positions[2][2] = {
            {std::round(x1 * r), std::round(y1 * r)},
            {std::round(x2 * r), std::round(y2 * r)}
        };
        int s = std::round(wh * r);

        /* Generate map */
        for(int obstacle = 0; obstacle < obstacles; obstacle++)
        {
            x1 = rand() % size[X];
            y1 = rand() % size[Y];
            x2 = std::min(size[X] - 1., x1 + rand() % s + 1);
            y2 = std::min(size[Y] - 1., y1 + rand() % s + 1);
            int z0 = std::max(rand() % size[Z], size[Z] >> 1);
            for(int x = x1; x < x2; x++)
                for(int y = y1; y < y2; y++)
                    for(int z = 0; z <= z0; z++)
                        map[x][y][z] = true;
        }
        
        /* Obstacles cannot cover robots */
        int xy = 2 * sz * r;
        for(int robot = 0; robot < 2; robot++)
        {
            x1 = std::max(positions[robot][0] - xy, 0.);
            y1 = std::max(positions[robot][1] - xy, 0.);
            x2 = std::min(positions[robot][0] + xy, size[X] * 1.);
            y2 = std::min(positions[robot][1] + xy, size[Y] * 1.);
            for(int x = x1; x < x2; x++)
                for(int y = y1; y < y2; y++)
                    for(int z = 0; z < size[Z]; z++)
                        map[x][y][z] = false;
        }
    }
}
