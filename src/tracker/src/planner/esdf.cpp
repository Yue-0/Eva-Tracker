/* @Author YueLin */

#include <cmath>

#include "planner/esdf.hpp"

#define request(array, size, type) new type**[size.x()];                       \
for(int x = 0; x < size.x(); x++)                                              \
{                                                                              \
    array[x] = new type*[size.y()];                                            \
    for(int y = 0; y < size.y(); y++)                                          \
        array[x][y] = new type[size.z()];                                      \
}

#define release(array, size)                                                   \
for(int x = 0; x < size.x(); x++)                                              \
{                                                                              \
    for(int y = 0; y < size.y(); y++)                                          \
        delete[] array[x][y];                                                  \
    delete[] array[x];                                                         \
}                                                                              \
delete[] array

namespace eva_tracker
{
    ESDF::~ESDF()
    {
        release(esdf, size);
    }

    ESDF::ESDF(double alpha, double beta, double distance, double r):
        resolution(r), maximum(-inf)
    {
        /* Initialize constants */
        beta = std::tan(beta / 2);
        alpha = std::tan(alpha / 2);
        offset << 0, distance * alpha, distance * beta;
        size << std::round(distance / r),
                std::round(2 * offset.y() / r),
                std::round(2 * offset.z() / r);
        size += Eigen::Vector3i::Constant(2);

        /* Construct the FoV's boundary */
        float y0 = size.y() / 2., z0 = size.z() / 2.;
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size.x(); x++)
        {
            bool d = x > distance / r;
            double ym = x * alpha, zm = x * beta;
            for(int y = 0; y < size.y(); y++)
            {
                bool a = d || std::fabs(y - y0) > ym;
                for(int z = 0; z < size.z(); z++)
                    data[x][y][z] = a || std::fabs(z - z0) > zm;
            }
        }
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    ESDF::ESDF(const Eigen::Vector3d& robot, double r, double expansion):
        resolution(r), maximum(-inf)
    {
        /* Initialize constants */
        offset = robot * (expansion / 2);
        size << std::round(robot.x() * expansion / r),
                std::round(robot.y() * expansion / r),
                std::round(robot.z() * expansion / r);

        /* Construct the robot's boundary */
        Eigen::Vector3d expand = (expansion - 1) / (2 * r) * robot;
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                for(int z = 0; z < size.z(); z++)
                    data[x][y][z] = !(
                        x - expand.x() > 0 && 
                        y - expand.y() > 0 && 
                        z - expand.z() > 0 &&
                        x + expand.x() < size.x() - 1 &&
                        y + expand.y() < size.y() - 1 &&
                        z + expand.z() < size.z() - 1
                    );
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    Eigen::Vector3d ESDF::argmax()
    {
        /* Initialize the maximum value */
        maximum = -inf;
        
        /* Find the maximum value */
        int x0 = 0, y0 = 0, z0 = 0;
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                for(int z = 0; z < size.z(); z++)
                    if(esdf[x][y][z] > maximum)
                        maximum = esdf[x0 = x][y0 = y][z0 = z];
        
        /* Return the index of the maximum value */
        return resolution * Eigen::Vector3d(x0, y0, z0);
    }

    void ESDF::build(bool*** data)
    {
        /* Initialize */
        std::vector<double> distance, f;
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                for(int z = 0; z < size.z(); z++)
                    esdf[x][y][z] = 0;

        /* Calculate the distance in the z direction */
        distance.resize(size.z());
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
            {
                for(int z = 0; z < size.z(); z++)
                    distance[z] = data[x][y][z]? 0: inf;
                for(int k = 1; k < size.z(); k++)
                    distance[k] = std::min(distance[k], distance[k - 1] + 1);
                for(int k = size.z() - 2; k >= 0; k--)
                    distance[k] = std::min(distance[k], distance[k + 1] + 1);
                for(int k = 0; k < size.z(); k++)
                    distance[k] *= distance[k];
                for(int z = 0; z < size.z(); z++)
                    esdf[x][y][z] = distance[z];
            }
        
        /* Calculate the distance in the y direction */
        f.resize(size.y());
        distance.resize(size.y());
        for(int x = 0; x < size.x(); x++)
            for(int z = 0; z < size.z(); z++)
            {
                for(int y = 0; y < size.y(); y++)
                    f[y] = esdf[x][y][z];
                transform(distance, f);
                for(int y = 0; y < size.y(); y++)
                    esdf[x][y][z] = distance[y];
            }

        /* Calculate the distance in the x direction */
        f.resize(size.x());
        distance.resize(size.x());
        for(int y = 0; y < size.y(); y++)
            for(int z = 0; z < size.z(); z++)
            {
                for(int x = 0; x < size.x(); x++)
                    f[x] = esdf[x][y][z];
                transform(distance, f);
                for(int x = 0; x < size.x(); x++)
                    esdf[x][y][z] = distance[x];
            }

        /* Map the result to Euclidean distance */
        for(int x = 0; x < size.x(); x++)
            for(int y = 0; y < size.y(); y++)
                for(int z = 0; z < size.z(); z++)
                    esdf[x][y][z] = resolution * std::sqrt(esdf[x][y][z]);
    }

    double ESDF::value(Eigen::Vector3d point, Eigen::Vector3d* grad) const
    {
        /* Initialize gradient */
        if(grad != nullptr)
            grad->setZero();

        /* Coordinate transformation */
        point += offset;
        point /= resolution;

        /* Bounds checking */
        if(point.minCoeff() < 0 || (size - point.cast<int>()).minCoeff() <= 1)
            return 0;

        /* Get neighboring points */
        int x[2], y[2], z[2];
        double u[2], v[2], w[2];
        *x = point.x(); *(x + 1) = *x + 1; 
        *y = point.y(); *(y + 1) = *y + 1;
        *z = point.z(); *(z + 1) = *z + 1;
        *u = point.x() - *x; *(u + 1) = 1 - *u;
        *v = point.y() - *y; *(v + 1) = 1 - *v;
        *w = point.z() - *z; *(w + 1) = 1 - *w;

        /* Cubic Linear interpolation */
        double d, s = 0;
        for(int i, j, k, n = 0; n < 8; n++)
        {
            k = n & 1;
            j = (n >> 1) & 1;
            i = (n >> 2) & 1;
            d = esdf[x[!i]][y[!j]][z[!k]];
            if(grad != nullptr)
            {
                grad->x() += v[j] * w[k] * d * (i? -1: 1);
                grad->y() += u[i] * w[k] * d * (j? -1: 1);
                grad->z() += u[i] * v[j] * d * (k? -1: 1);
            }
            s += u[i] * v[j] * w[k] * d;
        }
        return s;
    }

    void ESDF::transform(std::vector<double>& d, std::vector<double>& f) const
    {
        /* Initialize */
        int k, q;
        double s;
        int len = f.size();
        int* v = new int[len];
        double* z = new double[len + 1];
        
        /* Compute lower envelope */
        v[0] = 0;
        z[0] = -(z[1] = inf);
        for(k = q = 1; q < len; q++)
        {
            do {k--;}
            while(z[k] >= (s = (
                (f[q] + q * q) - (f[v[k]] + v[k] * v[k])
            ) / (2 * (q - v[k]))));
            v[++k] = q;
            z[k++] = s;
            z[k] = inf;
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
