/* @Author YueLin */

#pragma once

#include <cmath>
#include <chrono>
#include <limits>
#include <vector>

#define TIME 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(\
    std::chrono::system_clock::now().time_since_epoch()\
).count

namespace eva_tracker
{
    const double O = 1e-9;
    const double PI = std::acos(-1);
    const double INF = std::numeric_limits<double>::infinity();

    inline int sgn(double n)
    {return n > 0? 1: (n < 0? -1: 0);}
    
    inline double pow2(double n) {return n * n;}

    inline double clip(double rad)
    {return 2 * std::floor(0.5 - rad / (2 * PI)) * PI + rad;}

    template <typename type> class Point
    {
        public:
            double yaw;
            type x, y, z;

        public:
            Point()
            {
                x = 0; y = 0; z = 0; yaw = 0;
            }

            Point(type x0, type y0, type z0, double theta = 0)
            {
                x = x0; y = y0; z = z0; yaw = theta;
            }

            template <typename other> Point(Point<other> point)
            {
                x = point.x; y = point.y; z = point.z; yaw = point.yaw;
            }
        
            Point<type> operator+(const Point<type>& other)
            {
                return Point<type>(
                    x + other.x, y + other.y, z + other.z, clip(yaw + other.yaw)
                );
            }

            Point<type> operator-(const Point<type>& other)
            {
                return Point<type>(
                    x - other.x, y - other.y, z - other.z, clip(yaw - other.yaw)
                );
            }

            Point<double> operator*(const double n)
            {
                return Point<double>(x * n, y * n, z * n, yaw);
            }

            Point<double> operator/(const double n)
            {
                return *this * (1. / n);
            }

            bool operator==(const Point<type>& other)
            {
                return x == other.x && 
                       y == other.y && 
                       z == other.z && 
                       std::fabs(clip(yaw - other.yaw)) < O;
            }

            bool operator!=(const Point<type>& other)
            {
                return x != other.x ||
                       y != other.y ||
                       z != other.z ||
                       std::fabs(yaw - other.yaw) > O;
            }

            friend Point<double> operator*(const double n, Point<type> point)
            {
                return Point<double>(
                    point.x * n, point.y * n, point.z * n, clip(point.yaw * n)
                );
            }

        private:
            template <typename other>
            double distance2(const other x0, const other y0)
            {
                return pow2(x - x0) + pow2(y - y0);
            }
        
            template <typename other>
            double distance2(const Point<other> point)
            {
                return distance2(point.x, point.y) + pow2(z - point.z);
            }

        public:
            Point<int> integer(double ratio = 1.)
            {
                return Point<int>(
                    std::round(x * ratio),
                    std::round(y * ratio),
                    std::round(z * ratio), yaw
                );
            }

            template <typename other>
            double distance(const Point<other> point)
            {
                return std::sqrt(distance2(point));
            }

            template <typename other>
            double distance(const other x, const other y)
            {
                return std::sqrt(distance2(x, y));
            }
    };

    typedef std::vector<Point<double>> Path;
}
