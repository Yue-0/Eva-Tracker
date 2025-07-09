/* @Author YueLin */

#include <cmath>
#include <limits>

#include "planner/esdf.hpp"

#define request(arr, size, type) \
    new type**[size[X]];\
    for(int x = 0; x < size[X]; x++){\
        arr[x] = new type*[size[Y]];\
        for(int y = 0; y < size[Y]; y++)\
            arr[x][y] = new type[size[Z]];\
    }

#define release(arr, size) \
    for(int x = 0; x < size[X]; x++){\
        for(int y = 0; y < size[Y]; y++)\
            delete[] arr[x][y];\
        delete[] arr[x];\
    }delete[] arr

namespace eva_tracker
{
    ESDF::~ESDF()
    {
        release(esdf, size);
    }

    ESDF::ESDF(double alpha, double beta, double distance, double r)
    {
        /* Initialize constants */
        resolution = r;
        r = 1 / resolution;
        beta = std::tan(beta / 2);
        alpha = std::tan(alpha / 2);
        offset << 0, distance * alpha, distance * beta;
        size[X] = std::round(distance * r) + 2;
        size[Y] = std::round(offset.y() * r * 2) + 2;
        size[Z] = std::round(offset.z() * r * 2) + 2;

        /* Initialize the maximum value */
        maximum = -std::numeric_limits<double>::infinity();

        /* Construct the FoV's boundary */
        float y0 = size[Y] / 2., z0 = size[Z] / 2.;
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size[X]; x++)
        {
            bool d = x > distance * r;
            double ym = x * alpha, zm = x * beta;
            for(int y = 0; y < size[Y]; y++)
            {
                bool a = d || std::fabs(y - y0) > ym;
                for(int z = 0; z < size[Z]; z++)
                    data[x][y][z] = a || std::fabs(z - z0) > zm;
            }
        }
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    ESDF::ESDF(Eigen::Vector3d robot, double r, double expansion)
    {
        /* Initialize constants */
        resolution = r; r = 1 / r;
        offset = robot * (expansion / 2);
        size[X] = std::round(robot.x() * r * expansion);
        size[Y] = std::round(robot.y() * r * expansion);
        size[Z] = std::round(robot.z() * r * expansion);

        /* Initialize the maximum value */
        maximum = -std::numeric_limits<double>::infinity();

        /* Construct the robot's boundary */
        Eigen::Vector3d expand = 0.5 * (expansion - 1) * r * robot;
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                for(int z = 0; z < size[Z]; z++)
                    data[x][y][z] = !(
                        x - expand.x() > 0 && 
                        y - expand.y() > 0 && 
                        z - expand.z() > 0 &&
                        x + expand.x() < size[X] - 1 &&
                        y + expand.y() < size[Y] - 1 &&
                        z + expand.z() < size[Z] - 1
                    );
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    Eigen::Vector3d ESDF::argmax()
    {
        /* Initialize the maximum value */
        maximum = -std::numeric_limits<double>::infinity();
        
        /* Find the maximum value */
        int x0 = 0, y0 = 0, z0 = 0;
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                for(int z = 0; z < size[Z]; z++)
                    if(esdf[x][y][z] > maximum)
                        maximum = esdf[x0 = x][y0 = y][z0 = z];
        
        /* Return the index of the maximum value */
        return resolution * Eigen::Vector3d(x0, y0, z0);
    }

    Eigen::Vector3d ESDF::gradient(Eigen::Vector3d point)
    {
        /* Initialize gradient */
        Eigen::Vector3d g = Eigen::Vector3d::Zero();

        /* Coordinate transform */
        point = transform(point);

        /* Get neighboring points */
        int coordinate[6];
        double weights[6];
        interpolation(point, coordinate, weights);

        /* Bounds checking */
        int* x = coordinate;
        int* y = coordinate + 2;
        int* z = coordinate + 4;
        if(x[0] < 0 || x[0] >= size[X] ||
           x[1] < 0 || x[1] >= size[X] ||
           y[0] < 0 || y[0] >= size[Y] ||
           y[1] < 0 || y[1] >= size[Y] ||
           z[0] < 0 || z[0] >= size[Z] ||
           z[1] < 0 || z[1] >= size[Z]) return g;
        
        /* Linear interpolation */
        double d = 0;
        double* u = weights;
        double* v = weights + 2;
        double* w = weights + 4;
        for(int i, j, k, n = 0; n < 8; n++)
        {
            k = n & 1;
            j = (n >> 1) & 1;
            i = (n >> 2) & 1;
            d = esdf[x[!i]][y[!j]][z[!k]];
            g.x() += v[j] * w[k] * d * (i? -1: 1);
            g.y() += u[i] * w[k] * d * (j? -1: 1);
            g.z() += u[i] * v[j] * d * (k? -1: 1);
        }
        return g;
    }

    void ESDF::build(bool*** data)
    {
        /* Initialize */
        const double INF = std::numeric_limits<double>::infinity();
        std::vector<double> distance, f;
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                for(int z = 0; z < size[Z]; z++)
                    esdf[x][y][z] = 0;

        /* Calculate the distance in the z direction */
        distance.resize(size[Z]);
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
            {
                for(int z = 0; z < size[Z]; z++)
                    distance[z] = data[x][y][z]? 0: INF;
                dt(distance);
                for(int z = 0; z < size[Z]; z++)
                    esdf[x][y][z] = distance[z];
            }
        
        /* Calculate the distance in the y direction */
        f.resize(size[Y]);
        distance.resize(size[Y]);
        for(int x = 0; x < size[X]; x++)
            for(int z = 0; z < size[Z]; z++)
            {
                for(int y = 0; y < size[Y]; y++)
                    f[y] = esdf[x][y][z];
                dt(distance, f);
                for(int y = 0; y < size[Y]; y++)
                    esdf[x][y][z] = distance[y];
            }

        /* Calculate the distance in the x direction */
        f.resize(size[X]);
        distance.resize(size[X]);
        for(int y = 0; y < size[Y]; y++)
            for(int z = 0; z < size[Z]; z++)
            {
                for(int x = 0; x < size[X]; x++)
                    f[x] = esdf[x][y][z];
                dt(distance, f);
                for(int x = 0; x < size[X]; x++)
                    esdf[x][y][z] = distance[x];
            }

        /* Map the result to Euclidean distance */
        for(int x = 0; x < size[X]; x++)
            for(int y = 0; y < size[Y]; y++)
                for(int z = 0; z < size[Z]; z++)
                    esdf[x][y][z] = resolution * std::sqrt(esdf[x][y][z]);
    }

    double ESDF::value(Eigen::Vector3d point)
    {
        /* Get neighboring points */
        int coordinate[6];
        double weights[6];
        interpolation(point, coordinate, weights);

        /* Bounds checking */
        int* x = coordinate;
        int* y = coordinate + 2;
        int* z = coordinate + 4;
        if(x[0] < 0 || x[0] >= size[X] ||
           x[1] < 0 || x[1] >= size[X] ||
           y[0] < 0 || y[0] >= size[Y] ||
           y[1] < 0 || y[1] >= size[Y] ||
           z[0] < 0 || z[0] >= size[Z] ||
           z[1] < 0 || z[1] >= size[Z]) return 0;

        /* Linear interpolation */
        double d = 0;
        double* u = weights;
        double* v = weights + 2;
        double* w = weights + 4;
        for(int i, j, k, n = 0; n < 8; n++)
        {
            i = n >> 2; j = (n >> 1) & 1; k = n & 1;
            d += u[i] * v[j] * w[k] * esdf[x[!i]][y[!j]][z[!k]];
        }
        return d;
    }

    void ESDF::dt(std::vector<double>& dp)
    {
        const int len = dp.size();
        for(int k = 1; k < len; k++)
            dp[k] = std::min(dp[k], dp[k - 1] + 1);
        for(int k = len - 2; k >= 0; k--)
            dp[k] = std::min(dp[k], dp[k + 1] + 1);
        for(int k = 0; k < len; k++)
            dp[k] *= dp[k];
    }

    void ESDF::dt(std::vector<double>& d, std::vector<double>& f)
    {
        /* Initialize */
        int k, q;
        double s;
        int len = f.size();
        int* v = new int[len];
        double* z = new double[len + 1];
        const double INF = std::numeric_limits<double>::infinity();
        
        /* Compute lower envelope */
        v[0] = 0;
        z[0] = -(z[1] = INF);
        for(k = q = 1; q < len; q++)
        {
            do {k--;}
            while(z[k] >= (s = (
                (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
            ) / (2 * (q - v[k]))));
            v[++k] = q;
            z[k++] = s;
            z[k] = INF;
        }

        /* Fill in values of distance transform */
        for(k = q = 0; q < len; q++)
        {
            while(z[++k] < q);
            --k; d[q] = f[v[k]] + (q - v[k]) * (q - v[k]);
        }

        /* Release memory */
        delete[] z;
        delete[] v;
    }
}
