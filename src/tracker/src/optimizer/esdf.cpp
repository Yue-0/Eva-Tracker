/* @Author YueLin */

#include "optimizer/esdf.hpp"

#define request(arr, size, type) \
    new type**[size.x];\
    for(int x = 0; x < size.x; x++){\
        arr[x] = new type*[size.y];\
        for(int y = 0; y < size.y; y++)\
            arr[x][y] = new type[size.z];\
    }

#define release(arr, size) \
    for(int x = 0; x < size.x; x++){\
        for(int y = 0; y < size.y; y++)\
            delete[] arr[x][y];\
        delete[] arr[x];\
    }delete[] arr

namespace eva_tracker
{
    ESDF::~ESDF()
    {
        release(esdf, size);
    }

    ESDF::ESDF(Point<double> robot, double r, double expansion)
    {
        /* Initialize constants */
        maximum = -INF;
        resolution = r;
        offset = robot * (expansion / 2);
        size = robot.integer(expansion / r);
        Point<int> expand = robot.integer((expansion - 1) / (2 * r));

        /* Construct the robot's boundary */
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    data[x][y][z] = !(
                        x - expand.x > 0 && 
                        y - expand.y > 0 && 
                        z - expand.z > 0 &&
                        x + expand.x < size.x - 1 &&
                        y + expand.y < size.y - 1 &&
                        z + expand.z < size.z - 1
                    );
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    ESDF::ESDF(double alpha, double beta, double distance, double r)
    {
        /* Initialize constants */
        maximum = -INF;
        resolution = r;
        r = 1 / resolution;
        beta = std::tan(beta / 2);
        alpha = std::tan(alpha / 2);
        offset = Point<double>(0, distance * alpha, distance * beta) * 1.25;
        size = offset.integer(2 * r); size.x = std::round(distance * r * 1.25);

        /* Construct the FOV's boundary */
        float y0 = size.y / 2., z0 = size.z / 2.;
        bool*** data = request(data, size, bool);
        for(int x = 0; x < size.x; x++)
        {
            bool d = x > distance * r;
            double ym = x * alpha, zm = x * beta;
            for(int y = 0; y < size.y; y++)
            {
                bool a = d || std::fabs(y - y0) > ym;
                for(int z = 0; z < size.z; z++)
                    data[x][y][z] = a || std::fabs(z - z0) > zm;
            }
        }
        
        /* Build the ESDF and release memory */
        esdf = request(esdf, size, double);
        build(data); release(data, size);
    }

    Point<double> ESDF::gradient(Point<double> point)
    {
        Point<double> delta(0, 0, 0);

        /* Coordinate transform */
        point = transform(point);

        /* Bounds checking */
        if(point.x < 0 || point.x >= size.x ||
           point.y < 0 || point.y >= size.y ||
           point.z < 0 || point.z >= size.z) return delta;
        
        Point<double> grad;
        
        /* Calculate the gradient in the x direction */
        delta = Point<double>(1, 0, 0);
        grad.x = value(point + delta) - value(point - delta);

        /* Calculate the gradient in the y direction */
        delta = Point<double>(0, 1, 0);
        grad.y = value(point + delta) - value(point - delta);

        /* Calculate the gradient in the z direction */
        delta = Point<double>(0, 0, 1);
        grad.z = value(point + delta) - value(point - delta);

        return grad * 0.5;
    }

    Point<double> ESDF::argmax()
    {
        maximum = -INF;
        Point<int> index;
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    if(esdf[x][y][z] > maximum)
                    {
                        index.x = x;
                        index.y = y;
                        index.z = z;
                        maximum = esdf[x][y][z];
                    }
        return index * resolution;
    }

    double ESDF::max()
    {
        if(maximum < 0)
            for(int x = 0; x < size.x; x++)
                for(int y = 0; y < size.y; y++)
                    for(int z = 0; z < size.z; z++)
                        if(esdf[x][y][z] > maximum)
                            maximum = esdf[x][y][z];
        return maximum;
    }

    void ESDF::build(bool*** data)
    {
        /* Initialize */
        std::vector<double> distance, f;
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    esdf[x][y][z] = 0;

        /* Calculate the distance in the z direction */
        distance.resize(size.z);
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
            {
                for(int z = 0; z < size.z; z++)
                    distance[z] = data[x][y][z]? 0: INF;
                dt(distance);
                for(int z = 0; z < size.z; z++)
                    esdf[x][y][z] = distance[z];
            }
        
        /* Calculate the distance in the y direction */
        f.resize(size.y);
        distance.resize(size.y);
        for(int x = 0; x < size.x; x++)
            for(int z = 0; z < size.z; z++)
            {
                for(int y = 0; y < size.y; y++)
                    f[y] = esdf[x][y][z];
                dt(distance, f);
                for(int y = 0; y < size.y; y++)
                    esdf[x][y][z] = distance[y];
            }

        /* Calculate the distance in the x direction */
        f.resize(size.x);
        distance.resize(size.x);
        for(int y = 0; y < size.y; y++)
            for(int z = 0; z < size.z; z++)
            {
                for(int x = 0; x < size.x; x++)
                    f[x] = esdf[x][y][z];
                dt(distance, f);
                for(int x = 0; x < size.x; x++)
                    esdf[x][y][z] = distance[x];
            }

        /* Map the result to Euclidean distance */
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    esdf[x][y][z] = resolution * std::sqrt(esdf[x][y][z]);
    }

    double ESDF::value(Point<double> point)
    {
        /* Get 8 neighboring points */
        int x0 = point.x; int x1 = x0 + 1;
        int y0 = point.y; int y1 = y0 + 1;
        int z0 = point.z; int z1 = z0 + 1;

        /* Bounds checking */
        if(x0 < 0 || x1 >= size.x ||
           y0 < 0 || y1 >= size.y ||
           z0 < 0 || z1 >= size.z) return 0;

        /* Linear interpolation of x */
        double k = point.x - x0;
        double interpolation1[4] = {
            (1 - k) * esdf[x0][y0][z0] + k * esdf[x1][y0][z0],
            (1 - k) * esdf[x0][y1][z0] + k * esdf[x1][y1][z0],
            (1 - k) * esdf[x0][y0][z1] + k * esdf[x1][y0][z1],
            (1 - k) * esdf[x0][y1][z1] + k * esdf[x1][y1][z1]
        };

        /* Linear interpolation of y */
        k = point.y - y0;
        double interpolation2[2] = {
            (1 - k) * interpolation1[0b00] + k * interpolation1[0b10],
            (1 - k) * interpolation1[0b01] + k * interpolation1[0b11]
        };

        /* Linear interpolation of z */
        k = point.z - z0;
        return (1 - k) * interpolation2[0] + k * interpolation2[1];
    }

    Point<double> ESDF::transform(Point<double>& point)
    {
        return (point + offset) / resolution;
    }

    void ESDF::dt(std::vector<double>& d, std::vector<double>& f)
    {
        /* Initialize */
        int k, q;
        double s;
        const int len = f.size();
        double* z = new double[len + 1];
        int* v = new int[len];
        
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
            --k; d[q] = f[v[k]] + pow2(q - v[k]);
        }

        /* Release memory */
        delete[] z;
        delete[] v;
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
}
