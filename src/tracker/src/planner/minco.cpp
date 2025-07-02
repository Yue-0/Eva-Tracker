/* @Author: YueLin */

#include <cmath>

#include "planner/minco.hpp"

namespace eva_tracker
{
    Minco::Minco(): a(nullptr)
    {
        /* Initialize the factorial matrix */
        for(int i = *factorial[0] = 1; i < N; i++)
        {
            *factorial[i] = i ** factorial[i - 1];
            for(int j = 1; j <= i; j++)
                factorial[i][j] = *factorial[i] / *factorial[j];
        }

        /* Initialize matrix Q */
        q = Eigen::MatrixXi::Zero(S, S);
        for(int i = 0; i < S; i++)
            for(int j = 0; j < S; j++)
                q(i, j) = factorial[i + S][i] 
                        * factorial[j + S][j] / (i + j + 1);
    }

    Minco::~Minco()
    {
        if(a != nullptr) delete[] a;
    }

    void Minco::initialize(Eigen::MatrixXd& start, Eigen::MatrixXd& end, int p)
    {
        n = p;
        tail = end;
        head = start;

        /* Initialize boundary system */
        b.resize(n * N, D);
        if(a != nullptr) delete[] a;
        a = new double[n * N * (N * 2 + 1)];
        std::fill_n(a, n * N * (N * 2 + 1), 0.);

        /* Initialize the time vector */
        for(p = 0; p < N; p++)
            times[p].resize(n);
        times[0].setOnes();
    }

    void Minco::set(Eigen::MatrixXd& points, Eigen::VectorXd& durations)
    {
        /* Calculate times */
        for(int i = 1; i < N; i++)
            times[i] = times[i - 1].cwiseProduct(durations);
        
        /* Calculate matrix b */
        b.setZero();
        for(int i = 0; i < S; i++)
            b.row(i) = head.col(i).transpose();
        for(int i = 0; i < n - 1; i++)
            b.row(N * i + N - 1) = points.col(i).transpose();
        for(int i = 0; i < S; i++)
            b.row(N * n - S + i) = tail.col(i).transpose();
        
        /* Calculate array a */
        std::fill_n(a, n * N * (N * 2 + 1), 0.);
        for(int i = 0; i < S; i++)
            a[index(i, i)] = *factorial[i];
        for(int i = 0; i < n - 1; i++)
        {
            int idx, offset = N * i;
            
            for(int s = 0, len = S; s < S - 1; s++, len--)
            {
                idx = S + s + offset;
                for(int j = 0; j < len; j++)
                    a[index(idx, idx + j)] = times[j][i] * div(S + s + j, j);
                a[index(idx, idx + N)] = -*factorial[S + s];
            }

            idx = offset + N - 1;
            for(int j = 0; j < N; j++)
                a[index(idx, offset + j)] = times[j][i];

            for(int s = 0; s < S; s++)
            {
                idx = N + s + offset;
                for(int j = s; j < N; j++)
                    a[index(idx, offset + j)] = times[j - s][i] * div(j, j - s);
                a[index(idx, idx)] = -*factorial[s];
            }
        }
        for(int i = S, offset = N * n; i; i--)
            for(int j = i + S, s = 0; j; j--)
                a[index(offset - i, offset - j)] = times[s++][n - 1] 
                                                 * div(N - j, N - j + i - S);
        
        /* Banded LU factorization */
        const int M = n * N - 1;
        for(int k = 0; k < M; k++)
        {
            double c = a[index(k, k)];
            int s = std::min(k + N, M);
            for(int i = k + 1; i <= s; i++)
            {
                int idx = index(i, k);
                if(a[idx]) a[idx] /= c;
            }
            for(int j = k + 1; j <= s; j++)
                if((c = a[index(k, j)]))
                    for(int i = k + 1; i <= s; i++)
                    {
                        int idx = index(i, k);
                        if(a[idx]) a[index(i, j)] -= a[idx] * c;
                    }
        }

        /* Slove b */
        for(int j = 0; j <= M; j++)
        {
            int s = std::min(j + N, M);
            for(int i = j + 1; i <= s; i++)
            {
                int idx = index(i, j);
                if(a[idx]) b.row(i) -= a[idx] * b.row(j);
            }
        }
        for(int j = M; j >= 0; j--)
        {
            b.row(j) /= a[index(j, j)];
            for(int i = std::max(j - N, 0); i <= j - 1; i++)
            {
                int idx = index(i, j);
                if(a[idx]) b.row(i) -= a[idx] * b.row(j);
            }
        }
    }

    void Minco::get(Trajectory* path)
    {
        std::vector<double> t(n);
        std::vector<Eigen::MatrixXd> c(n);
        for(int i = 0; i < n; i++)
        {
            t[i] = times[1][i];
            c[i] = b.block(i * N, 0, N, D).transpose().rowwise().reverse();
        }
        path->set(t, c);
    }

    void Minco::propogate(Eigen::VectorXd& tau, 
                          Eigen::VectorXd& input,
                          Eigen::VectorXd& output)
    {
        for(int i = 0; i < n; i++)
            output[i] = tau[i] > 0
                      ? input[i] * (1. + tau[i])
                      : input[i] * (1. - tau[i])
                      / std::pow(tau[i] * (tau[i] * 0.5 - 1.) + 1., 2);
    }

    void Minco::propogate(Eigen::MatrixXd dc, Eigen::VectorXd& dt0,
                          Eigen::MatrixXd& dp, Eigen::VectorXd& dt, bool end)
    {
        /* Propogate gradient to points */
        int rows = N * n - 1;
        dp.resize(D, n - !end);
        for(int j = 0; j <= rows; j++)
        {
            int s = std::min(j + N, rows);
            dc.row(j) /= a[index(j, j)];
            for(int i = j + 1; i <= s; i++)
            {
                int idx = index(j, i);
                if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
            }
        }
        for(int j = rows; j >= 0; j--)
        {
            int s = std::max(j - N, 0);
            for(int i = s; i <= j - 1; i++)
            {
                int idx = index(j, i);
                if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
            }
        }
        rows = n - !end;
        for(int i = 0; i < rows; i++)
            dp.col(i) = dc.row(N * (i + 1) - 1).transpose();
        
        /* Propogate gradient to times */
        dt.resize(n);
        int offset = 0;
        Eigen::MatrixXd m = Eigen::MatrixXd::Zero(N, D);
        for(int p = 0; p < n - 1; offset = (m.setZero(), N * ++p))
        {
            for(int i = 1, s = N - 1; i < N - 1; s--, i++)
            {
                int idx = (i + S - 1) % N;
                for(int j = 0; j < s; j++)
                    m.row(idx) -= times[j][p] 
                                * div(i + j, j) 
                                * b.row(offset + i + j);
            }
            m.row(S - 1) = m.row(S);
            dt[p] = m.cwiseProduct(dc.block(offset + S, 0, N, D)).sum();
        }
        for(int i = 1, s = N - 1; i <= S; s--, i++)
            for(int j = 0; j < s; j++)
                m.row(i - 1) -= div(i + j, j)
                              * times[j][n - 1]
                              * b.row(offset + i + j);
        dt[n - 1] = m.block(0, 0, S, D).cwiseProduct(
            dc.block(n * N - S, 0, S, D)
        ).sum(); dt += dt0;
    }

    double Minco::cost(Eigen::MatrixXd& dc, Eigen::VectorXd& dt, double lambda)
    {
        /* Calculate trajectory cost partial gradient by coefficients */
        dc.resize(N * n, D); dc.setZero();
        for(int p = 0; p < n; p++)
        {
            int idx = p * N + S;
            for(int i = 0; i < S; i++)
                for(int j = 0; j < S; j++)
                    dc.row(idx + i) += 2 * lambda * q(i, j) 
                                     * times[i + j + 1][p] 
                                     * b.row(idx + j);
        }
        
        /* Calculate trajectory cost and its partial gradient by times */
        dt.resize(n);
        dt.setZero();
        double e, energy = 0;
        for(int p = 0; p < n; p++)
        {
            int idx = p * N + S;
            for(int i = 0; i < S; i++)
                for(int j = i, t = i; j >= 0; j--)
                {
                    e = lambda
                      * q(i, i - j) * (j? 2: 1)
                      * b.row(idx + i).dot(b.row(idx + i - j)); 
                    dt[p] += (t + 1) * e * times[t][p];
                    energy += e * times[++t][p];
                }
        }
        return energy;
    }

    void Minco::diffeomorphism(const Eigen::VectorXd& input,
                               Eigen::VectorXd& output, bool forward)
    {
        output.resize(n);

        /* Forward tau -> t */
        if(forward) for(int i = 0; i < n; i++)
            output[i] = input[i] > 0.
                      ? (0.5 * input[i] + 1.) * input[i] + 1.
                      : 1. / ((0.5 * input[i] - 1.) * input[i] + 1.);
        
        /* Backward t -> tau */
        else for(int i = 0; i < n; i++)
            output[i] = input[i] > 1.
                      ? std::sqrt(2. * input[i] - 1.) - 1.
                      : 1. - std::sqrt(2. / input[i] - 1.);
    }

    int Trajectory::index(double* time)
    {
        int idx = 0;
        const int N = size() - 1;
        while(idx < N && *time > times[idx])
            *time -= times[idx++];
        return idx;
    }

    Eigen::VectorXd Trajectory::derivative(double time, 
                                           const int n, 
                                           const int idx)
    {
        int s = D - n;
        double p = time;
        Eigen::VectorXd d = coefficients[idx].col(s);
        for(int order = n + 1; s; ++order, p *= time)
        {
            int c = 1;
            for(int i = 0; i < n; i++)
                c *= order + i;
            d += c * p * coefficients[idx].col(--s);
        }
        return d;
    }

    double Trajectory::duration()
    {
        double d = 0;
        for(double t: times)
            d += t;
        return d;
    }
}
