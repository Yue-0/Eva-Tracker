/* @Author: YueLin */

#include <cmath>
#include <limits>
#include <cstdlib>

#include "simulator/map.hpp"

namespace simulator
{
    Map::~Map()
    {
        for(int x = 0; x < size.x(); x++)
        {
            for(int y = 0; y < size.y(); y++)
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

    Map::Map(double x, double y, double z, double r): resolution(r)
    {
        /* Initalize */
        size0 << x, y, z;
        size = (size0 / r).cast<int>();
        size0 -= r * Eigen::Vector3d::Ones();

        /* Apply for memory */
        map = new bool**[size.x()];
        exp = new bool**[size.x()];
        sdf = new double*[size.x()];
        for(int i = 0; i < size.x(); i++)
        {
            map[i] = new bool*[size.y()];
            exp[i] = new bool*[size.y()];
            sdf[i] = new double[size.y()];
            for(int j = 0; j < size.y(); j++)
            {
                map[i][j] = new bool[size.z()];
                exp[i][j] = new bool[size.z()];
            }
        }

        /* Initialize map */
        clear();
    }

    void Map::clear()
    {
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
            {
                sdf[x][y] = 0;
                for(int z = 0; z < size.z(); z++)
                    map[x][y][z] = exp[x][y][z] = false;
            }
    }

    void Map::distance()
    {
        /* Initialize */
        const double INF = std::numeric_limits<double>::infinity();
        std::vector<double> distance, f;
        std::vector<std::vector<double>> negative(
            size.x(), std::vector<double>(size.y(), 0)
        );
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                sdf[x][y] = 0;
        
        /* Calculate the distance in the y direction */
        distance.resize(size.y());
        for(int x = 0; x < size.x(); x++)
        {
            for(int y = 0; y < size.y(); y++)
                distance[y] = exp[x][y][0]? 0: INF;
            for(int y = 1; y < size.y(); y++)
                distance[y] = std::min(distance[y], distance[y - 1] + 1);
            for(int y = size.y() - 2; y >= 0; y--)
                distance[y] = std::min(distance[y], distance[y + 1] + 1);
            for(int y = 0; y < size.y(); y++)
                distance[y] *= distance[y];
            for(int y = 0; y < size.y(); y++)
                sdf[x][y] = distance[y];
        }
        for(int x = 0; x < size.x(); x++)
        {
            for(int y = 1; y < size.y(); y++)
                distance[y] = exp[x][y][0]? INF: 0;
            distance[0] = distance[size.y() - 1] = 0;
            for(int y = 1; y < size.y(); y++)
                distance[y] = std::min(distance[y], distance[y - 1] + 1);
            for(int y = size.y() - 2; y >= 0; y--)
                distance[y] = std::min(distance[y], distance[y + 1] + 1);
            for(int y = 0; y < size.y(); y++)
                distance[y] *= distance[y];
            for(int y = 0; y < size.y(); y++)
                negative[x][y] = distance[y];
        }

        /* Calculate the distance in the y direction */
        f.resize(size.x());
        distance.resize(size.x());
        int* v = new int[size.x()];
        double* z = new double[size.x() + 1];
        for(int k, q, y = 0; y < size.y(); y++)
        {
            double s;
            v[0] = 0; z[0] = -(z[1] = INF);
            for(int x = 0; x < size.x(); x++)
                f[x] = sdf[x][y];
            for(k = q = 1; q < size.x(); q++)
            {
                do {k--;}
                while(z[k] >= (s = (
                    (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
                ) / (2 * (q - v[k]))));
                v[++k] = q;
                z[k++] = s;
                z[k] = INF;
            }
            for(k = q = 0; q < size.x(); q++)
            {
                while(z[++k] < q);
                --k; distance[q] = f[v[k]] + (q - v[k]) * (q - v[k]);
            }
            for(int x = 0; x < size.x(); x++)
                sdf[x][y] = distance[x];
        }
        for(int k, q, y = 0; y < size.y(); y++)
        {
            double s;
            v[0] = 0; z[0] = -(z[1] = INF);
            for(int x = 0; x < size.x(); x++)
                f[x] = negative[x][y];
            for(k = q = 1; q < size.x(); q++)
            {
                do {k--;}
                while(z[k] >= (s = (
                    (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
                ) / (2 * (q - v[k]))));
                v[++k] = q;
                z[k++] = s;
                z[k] = INF;
            }
            for(k = q = 0; q < size.x(); q++)
            {
                while(z[++k] < q);
                --k; distance[q] = f[v[k]] + (q - v[k]) * (q - v[k]);
            }
            for(int x = 0; x < size.x(); x++)
                negative[x][y] = distance[x];
        }
        delete[] z;
        delete[] v;

        /* Map the result to Euclidean distance */
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                sdf[x][y] = ! exp[x][y][0]
                            ? resolution * std::sqrt(sdf[x][y])
                            : resolution * -std::sqrt(negative[x][y]);
    }

    void Map::expand(double sz)
    {
        expansion = sz;
        int dist = std::round(sz / resolution);
        for(int x0 = 0; x0 < size.x(); x0++)
            for(int y0 = 0; y0 < size.y(); y0++)
                for(int z0 = 0; z0 < size.z(); z0++)
                    if(map[x0][y0][z0])
                    {
                        int x1 = std::max(x0 - dist, 0);
                        int y1 = std::max(y0 - dist, 0);
                        int z1 = std::max(z0 - dist, 0);
                        int x2 = std::min(x0 + dist, size.x() - 1);
                        int y2 = std::min(y0 + dist, size.y() - 1);
                        int z2 = std::min(z0 + dist, size.z() - 1);
                        for(int x = x1; x <= x2; x++)
                            for(int y = y1; y <= y2; y++)
                                for(int z = z1; z <= z2; z++)
                                    exp[x][y][z] = true;
                    }
        for(int x = 0; x < size.x(); x++)
            exp[x][0][0] = exp[size.x() - 1][0][0] = true;
        for(int y = 0; y < size.y(); y++)
            exp[0][y][0] = exp[0][size.y() - 1][0] = true;
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
            x1 = rand() % size.x();
            y1 = rand() % size.y();
            x2 = std::min(size.x() - 1., x1 + rand() % s + 1);
            y2 = std::min(size.y() - 1., y1 + rand() % s + 1);
            int z0 = std::max(rand() % size.z(), size.z() >> 1);
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
            x2 = std::min(positions[robot][0] + xy, size.x() * 1.);
            y2 = std::min(positions[robot][1] + xy, size.y() * 1.);
            for(int x = x1; x < x2; x++)
                for(int y = y1; y < y2; y++)
                    for(int z = 0; z < size.z(); z++)
                        map[x][y][z] = false;
        }
    }
}
