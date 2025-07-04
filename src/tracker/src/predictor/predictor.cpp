/* @Author: YueLin */

#include "predictor/predictor.hpp"

namespace eva_tracker
{
    Predictor::Predictor(Bezier* b): m(b->n >> 1), bezier(b)
    {
        /* Initialize factorial */
        int n = b->n;
        int c = n * 2 - 2;
        factorial = new int[c];
        for(int i = *factorial = 1; i < c; i++)
            factorial[i] = i * factorial[i - 1];

        /* Initialize matrix and vector */
        Eigen::MatrixXd a = Eigen::MatrixXd::Zero((n / 2 + 1) * 3, 3 * (1 + n));
        c = a.rows() + (n + 1) * 3;
        vector = Eigen::VectorXd::Zero(c);
        matrix = Eigen::MatrixXd::Zero(c, c);

        /* Calculate matrix */
        for(int i = 0; i <= n; i++)
            for(int j = 0; j <= n; j++)
            {
                c = combination(n - 2, i) * combination(n - 2, j);
                if(c) matrix(i, j) += c * integral(i + j, 2 * n - i - j - 4);

                c = combination(n - 2, i - 2) * combination(n - 2, j - 2);
                if(c) matrix(i, j) += c * integral(i + j - 4, 2 * n - i - j);

                c = combination(n - 2, i) * combination(n - 2, j - 2)
                  + combination(n - 2, i - 2) * combination(n - 2, j)
                  + combination(n - 2, i - 1) * combination(n - 2, j - 1) * 4;
                if(c) matrix(i, j) += c * integral(
                    i + j - 2, 2 * n - i - j - 2
                );

                c = combination(n - 2, i) * combination(n - 2, j - 1)
                  + combination(n - 2, i - 1) * combination(n - 2, j);
                if(c) matrix(i, j) -= 2 * c * integral(
                    i + j - 1, 2 * n - i - j - 3
                );
                
                c = combination(n - 2, i - 1) * combination(n - 2, j - 2)
                  + combination(n - 2, i - 2) * combination(n - 2, j - 1);
                if(c) matrix(i, j) -= 2 * c * integral(
                    i + j - 3, 2 * n - i - j - 1
                );
            }
        c = n + 1;
        matrix.block(0, 0, c, c) *= n * n * (n - 1) * (n - 1) / std::pow(
            2 * b->duration, n << 1
        );
        matrix.block(c, c, c, c) = matrix.block(0, 0, c, c);
        matrix.block(c << 1, c << 1, c, c) = matrix.block(0, 0, c, c);
        
        /* Constraints */
        n = n / 2 + 1;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < c; j++)
                a(i, j) = b->bernstein(i * b->time - b->duration, j);
        a.block(n, c, n, c) = a.block(0, 0, n, c);
        a.block(n << 1, c << 1, n, c) = a.block(0, 0, n, c);
        matrix.block(0, c * 3, c * 3, n * 3) = a.transpose();
        matrix.block(c * 3, 0, n * 3, c * 3) = a;

        /* Calculate the inverse matrix */
        matrix = matrix.inverse().topRows(c * 3);
    }

    void Predictor::predict(const Eigen::Matrix3Xd& observation)
    {
        /* Get observation vector */
        vector.segment(m * 6 + 3, m + 1) = observation.row(0).transpose();
        vector.segment(m * 7 + 4, m + 1) = observation.row(1).transpose();
        vector.segment(m * 8 + 5, m + 1) = observation.row(2).transpose();
        
        /* Solve KKT point */
        Eigen::VectorXd kkt = matrix * vector;

        /* Set control points */
        bezier->control.row(0) = kkt.head(m * 2 + 1).transpose();
        bezier->control.row(1) = kkt.segment(m * 2 + 1, m * 2 + 1).transpose();
        bezier->control.row(2) = kkt.segment(m * 4 + 2, m * 2 + 1).transpose();
    }
}
