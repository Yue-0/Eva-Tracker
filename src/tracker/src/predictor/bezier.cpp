/* @Author YueLin */

#include "predictor/bezier.hpp"

namespace eva_tracker
{
    Bezier::Bezier(int num, double t)
    : n(num - 1), time(t), duration(t * (num >> 1))
    {
        /* Initialize control points */
        control.resize(3, num);

        /* Precompute factorials */
        factorial = new int[num];
        for(int i = *factorial = 1; i < num; i++)
            factorial[i] = i * factorial[i - 1];
    }

    Eigen::Matrix3Xd Bezier::trajectory()
    {
        int num = n / 2 + 1;
        Eigen::Matrix3Xd path(3, num);
        for(int p = 0; p < num; p++)
            path.col(p) = (*this)[time * p];
        return path;
    }

    Eigen::Vector3d Bezier::derivative(double t)
    {
        Eigen::Vector3d point(0, 0, 0);
        t = std::max(std::min(t, duration), -duration);
        for(int p = 0; p <= n; p++)
            point += control.col(p) * (
                bernstein(t, p - 1, n - 1) - bernstein(t, p, n - 1)
            );
        return point * (n / (duration * 2));
    }

    Eigen::Vector3d Bezier::operator[](double t)
    {
        Eigen::Vector3d point(0, 0, 0);
        t = std::max(std::min(t, duration), -duration);
        for(int p = 0; p <= n; p++)
            point += control.col(p) * bernstein(t, p);
        return point;
    }
}
