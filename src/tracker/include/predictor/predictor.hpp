/* @Author YueLin */

#include "predictor/bezier.hpp"

namespace eva_tracker
{
    class Predictor
    {
        private:
            const int m;
            Bezier* bezier;
            Eigen::VectorXd vector;
            Eigen::MatrixXd inverse;

        public:
            Predictor(Bezier* b): m(b->n >> 1), bezier(b)
            {
                /* Initialize */
                int n = b->n;
                Eigen::MatrixXd cm = Eigen::MatrixXd::Zero(
                    (n / 2 + 1) * 3, 3 * (1 + n)
                );
                int c = cm.rows() + cm.cols();
                vector = Eigen::VectorXd::Zero(c);
                Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(c, c);

                /* Calculate the coefficient matrix */
                for(int i = 0; i <= n; i++)
                for(int j = 0; j <= n; j++)
                for(int k = 0; k <= 4; k++)
                {
                    c = 0;
                    for(int p = 0; p < 3; p++)
                        c += combination(2, p) 
                           * combination(2, k - p)
                           * combination(n - 2, i - p) 
                           * combination(n - 2, j - k + p);
                    if(c)
                        matrix(i, j) += c * (k & 1? -1: 1)
                                          * factorial(i + j - k) 
                                          * factorial(2 * n - i - j + k - 4);
                }
                c = n + 1;
                matrix.topLeftCorner(c, c) *= std::pow(n * (n - 1), 2) / (
                    factorial(n * 2 - 3) * std::pow(2 * b->duration, 3)
                );
                matrix.block(c, c, c, c) = matrix.topLeftCorner(c, c);
                matrix.block(c << 1, c << 1, c, c) = matrix.topLeftCorner(c, c);
                
                /* Calculate the constraint matrix */
                n = n / 2 + 1;
                for(int i = 0; i < n; i++)
                    for(int j = 0; j < c; j++)
                        cm(i, j) = b->bernstein(i * b->time - b->duration, j);
                cm.block(n, c, n, c) = cm.topLeftCorner(n, c);
                cm.bottomRightCorner(n, c) = cm.topLeftCorner(n, c);

                /* Merge coefficient matrix and constraint matrix */
                n *= 3;
                c *= 3;
                matrix.bottomLeftCorner(n, c) = cm;
                matrix.topRightCorner(c, n) = cm.transpose();

                /* Calculate the inverse matrix */
                inverse = matrix.inverse().topRows(c);
            }

        public:
            void predict(const Eigen::Matrix3Xd& obs)
            {
                /* Get observation vector */
                int n = m + 1;
                const int i = m * 6 + 3;
                for(int d = 0; d < 3; d++)
                    vector.segment(i + d * n, n) = obs.row(d).transpose();
                
                /* Solve KKT point */
                Eigen::VectorXd x = inverse * vector;

                /* Set control points */
                n += m;
                for(int d = 0; d < 3; d++)
                    bezier->control.row(d) = x.segment(d * n, n).transpose();
            }
    };
}
