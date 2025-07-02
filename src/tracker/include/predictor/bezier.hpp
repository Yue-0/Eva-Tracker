/* @Author YueLin */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace eva_tracker
{
    class Bezier
    {
        public:
            int n;
            int* factorial;
            double time, duration;
            Eigen::Matrix3Xd control;

        public:
            Bezier(int, double);
            ~Bezier() {delete[] factorial;}
            
            Eigen::Matrix3Xd trajectory();  // b(0) ~ b(duration)  step: time
            Eigen::Vector3d derivative(double);  // b'(t)
            Eigen::Vector3d operator[](double);  // b(t)

            /* Bernstein function */
            double bernstein(double t, int i, int m)
            {
                if(i < 0 || i > m) return 0;
                return factorial[m] / (factorial[i] * factorial[m - i])
                     * std::pow(duration - t, m - i)
                     * std::pow(duration + t, i)
                     / std::pow(2 * duration, m)
                     ;
            }
            double bernstein(double t, int i) {return bernstein(t, i, n);}
    };
}
