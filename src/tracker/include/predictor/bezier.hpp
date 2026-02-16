/* @Author YueLin */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

#include "predictor/factorial.h"

namespace eva_tracker
{
    class Bezier
    {
        public:
            const int n;                // Degree
            const double time;          // Time interval between adjacent points
            const double duration;      // The duration of a HALF Bezier curve
            Eigen::Matrix3Xd control;   // Control points

        public:
            /**
             * Initialize a Bezier curve.
             * 
             * @param num Number of control points.
             * @param t   Time interval between adjacent points.
             */
            Bezier(int num, double t): 
                n(num - 1), time(t), duration(t * (num >> 1)){
                control.resize(3, num);
            }

            /**
             * Bernstein function.
             * 
             * @param t Time.
             * @param i Order.
             * @return  The value of the `bernstein` function with `m = n`.
             * 
             * \sa bernstein(double t, int i, int m).
             */
            double bernstein(double t, int i)
            {
                return bernstein(t, i, n);
            }

            /**
             * Return the point of the Bezier curve at time `t`.
             * 
             * If `t` is greater than `duration`, 
             *  return the point at `duration`.
             * 
             * If `t` is less than `-duration`, 
             *  return the point at `-duration`.
             */
            Eigen::Vector3d operator[](double t)
            {
                Eigen::Vector3d point = Eigen::Vector3d::Zero();
                t = std::max(std::min(t, duration), -duration);
                for(int p = 0; p <= n; p++)
                    point += control.col(p) * bernstein(t, p);
                return point;
            }

            /**
             * Return the derivative of the Bezier curve at time `t`.
             * 
             * If `t` is greater than `duration`, 
             *  return the derivative at `duration`.
             * 
             * If `t` is less than `-duration`, 
             *  return the derivative at `-duration`.
             */
            Eigen::Vector3d derivative(double t)
            {
                Eigen::Vector3d point = Eigen::Vector3d::Zero();
                t = std::max(std::min(t, duration), -duration);
                for(int p = 0; p <= n; p++)
                    point += control.col(p) * (
                        bernstein(t, p - 1, n - 1) - bernstein(t, p, n - 1)
                    );
                return point * (n / (duration * 2));
            }
            
            /**
             * Obtain the trajectory of the Bezier curve, 
             * with the first point at `t = 0` 
             * and the last point at `t = duration`, 
             * and the time interval between each two points being `time`.
             * 
             * @return a Eigen::Matrix3Xd with `num / 2 + 1` columns.
             */
            Eigen::Matrix3Xd trajectory()
            {
                int num = n / 2 + 1;
                Eigen::Matrix3Xd path(3, num);
                for(int p = 0; p < num; p++)
                    path.col(p) = (*this)[time * p];
                return path;
            }

        private:
            /* Extended Bernstein function. */
            double bernstein(double t, int i, int m)
            {
                return (i < 0 || i > m)
                      ? 0: combination(m, i) 
                      / std::pow(2 * duration, m)
                      * std::pow(duration + t, i)
                      * std::pow(duration - t, m - i);
            }
    };
}
