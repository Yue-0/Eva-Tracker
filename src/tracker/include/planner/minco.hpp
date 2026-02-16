/* @Author YueLin */

#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "predictor/factorial.h"

namespace eva_tracker
{
    class Trajectory
    {
        private:
            const int order;
        
        private:
            std::vector<double> times;
            std::vector<Eigen::MatrixXd> coefficients;
        
        public:
            Trajectory(int s): order(s) {}
        
        public:
            /* Get the total duration of the trajectory */
            double duration() const
            {
                double d = 0;
                for(double t: times)
                    d += t;
                return d;
            }

            /* Get the point on the trajectory at time */
            Eigen::VectorXd pos(double time) {return derivative(time, 0);}
            
            /* Get the velocity on the trajectory at time */
            Eigen::VectorXd vel(double time) {return derivative(time, 1);}

            /* Get the acceleration on the trajectory at time */
            Eigen::VectorXd acc(double time) {return derivative(time, 2);}

            /* Get the jerk on the trajectory at time */
            Eigen::VectorXd jerk(double time) {return derivative(time, 3);}

            /* Clear trajectory */
            void clear() {times.clear(); coefficients.clear();}

            /* Get the number of pieces */
            int size() const {return times.size();}

            /* Set durations and coefficients */
            void set(const std::vector<double>& t, 
                     const std::vector<Eigen::MatrixXd>& c)
            {
                times.assign(t.begin(), t.end());
                coefficients.assign(c.begin(), c.end());
            }

        private:
            Eigen::VectorXd derivative(double time, const int n) const
            {
                /* Calculate index */
                int idx = 0;
                const int m = size() - 1;
                while(idx < m && time > times[idx])
                    time -= times[idx++];
                
                /* Calculate derivative */
                double p = time;
                int s = order - n;
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
    };

    template<int dim> class Minco
    {
        private:
            int n;                      // The number of pieces
            double* a;                  // Banded system
            const int s;                // The order of optimization
            Eigen::MatrixXi q;          // q(i,j) = (i+s)!(j+s)! / (i!j!(i+j+1))
        
        private:
            Eigen::MatrixXd times;
            Eigen::Matrix<double, -1, dim> b;
            Eigen::Matrix<double, dim, -1> head, tail;
        
        public:
            Minco(int od): a(nullptr), s(od)
            {
                /* Initialize matrix Q */
                q = Eigen::MatrixXi::Zero(s, s);
                for(int i = 0; i < s; i++)
                    for(int j = 0; j < s; j++)
                        q(i, j) = factorial(i + s, i) 
                                * factorial(j + s, j) / (i + j + 1);
            }

            ~Minco() {if(a != nullptr) delete[] a;}
        
            /* Initialize Minco */
            void initialize(int num, Eigen::Matrix<double, dim, -1>* boundary)
            {
                n = num;
                head = *boundary;
                tail = *(boundary + 1);

                /* Initialize boundary system */
                b.resize(2 * n * s, dim);
                if(a != nullptr) delete[] a;
                a = new double[n * s * (s * 4 + 1) * 2];
                std::fill_n(a, n * s * (s * 4 + 1) * 2, 0.);

                /* Initialize the time vector */
                times.resize(n, s * 2);
                times.col(0).setOnes();
            }
            
            /* Set points and times of Minco */
            void set(const Eigen::VectorXd& durations,
                     const Eigen::Matrix<double, dim, -1>& points)
            {
                /* Calculate times */
                int idx = s << 1;
                for(int i = 1; i < idx; i++)
                    times.col(i) = times.col(i - 1).cwiseProduct(durations);
                
                /* Calculate matrix b */
                b.setZero();
                for(int i = 0; i < s; i++)
                    b.row(i) = head.col(i).transpose();
                for(int i = 0; i < n - 1; i++)
                    b.row(s * (i + 1) * 2 - 1) = points.col(i).transpose();
                for(int i = 0; i < s; i++)
                    b.row(s * (n * 2 - 1) + i) = tail.col(i).transpose();
                
                /* Calculate array a */
                std::fill_n(a, n * s * (s * 4 + 1) * 2, 0.);
                for(int i = 0; i < s; i++)
                    a[index(i, i)] = factorial(i);
                for(int i = 0; i < n - 1; i++)
                {
                    const int offset = s * i * 2;
                    for(int p = 0, len = s; p < s - 1; p++, len--)
                    {
                        idx = s + p + offset;
                        for(int j = 0; j < len; j++)
                            a[index(idx, idx + j)] = times(i, j)
                                                   * factorial(s + p + j, j);
                        a[index(idx, idx + s * 2)] = -factorial(s + p);
                    }
                    idx = offset + s * 2 - 1;
                    for(int j = 0; j < (s << 1); j++)
                        a[index(idx, offset + j)] = times(i, j);
                    for(int p = 0; p < s; p++)
                    {
                        idx = 2 * s + p + offset;
                        for(int j = p; j < (s << 1); j++)
                            a[index(idx, offset + j)] = times(i, j - p)
                                                      * factorial(j, j - p);
                        a[index(idx, idx)] = -factorial(p);
                    }
                }
                for(int i = s, offset = s * n * 2; i; i--)
                    for(int j = i + s, p = 0; j; j--, p++)
                        a[index(offset - i, offset - j)] = times(n - 1, p) 
                                                         * factorial(2 * s - j, 
                                                                     s + i - j);
                
                /* Banded LU factorization */
                const int m = n * s * 2 - 1;
                for(int k = 0; k < m; k++)
                {
                    double c = a[index(k, k)];
                    idx = std::min(k + s * 2, m);
                    for(int i = k + 1; i <= idx; i++)
                        a[index(i, k)] /= c;
                    for(int j = k + 1; j <= idx; j++)
                        if((c = a[index(k, j)]))
                            for(int i = k + 1; i <= idx; i++)
                                a[index(i, j)] -= a[index(i, k)] * c;
                }

                /* Slove b */
                for(int j = 0; j <= m; j++)
                {
                    idx = std::min(j + s * 2, m);
                    for(int i = j + 1; i <= idx; i++)
                        b.row(i) -= a[index(i, j)] * b.row(j);
                }
                for(int j = m; j >= 0; j--)
                {
                    b.row(j) /= a[index(j, j)];
                    for(int i = std::max(j - s * 2, 0); i <= j - 1; i++)
                        b.row(i) -= a[index(i, j)] * b.row(j);
                }
            }
            
            /* Set the endpoint */
            void set(Eigen::Matrix<double, dim, 1>& endpoint) 
            {
                tail.col(0) = endpoint;
            }

            /* Get Minco trajectory */
            void get(Trajectory* path) const
            {
                std::vector<double> t(n);
                std::vector<Eigen::MatrixXd> c(n);
                for(int i = 0; i < n; i++)
                {
                    t[i] = times(i, 1);
                    c[i] = b.middleRows(
                        i * s * 2, 2 * s
                    ).transpose().rowwise().reverse();
                }
                path->set(t, c);
            }

            /* Propagate the gradient of t to tau */
            void propogate(const Eigen::VectorXd& tau, 
                           const Eigen::VectorXd& input, 
                           Eigen::VectorXd& output) const
            {
                for(int i = 0; i < n; i++)
                    output[i] = tau[i] > 0
                              ? input[i] * (1. + tau[i])
                              : input[i] * (1. - tau[i])
                              / std::pow(tau[i] * (tau[i] * 0.5 - 1.) + 1., 2);
            }

            /* Propogate gradients to points and times */
            void propogate( 
                Eigen::Matrix<double, -1, dim> dc, const Eigen::VectorXd& dt0,
                Eigen::Matrix<double, dim, -1>& dp, Eigen::VectorXd& dt,
                bool end
            ) const
            {
                /* Propogate gradient to points */
                int rows = s * n * 2 - 1;
                dp.resize(dim, n - !end);
                for(int j = 0; j <= rows; j++)
                {
                    int r = std::min(j + s * 2, rows);
                    dc.row(j) /= a[index(j, j)];
                    for(int i = j + 1; i <= r; i++)
                    {
                        int idx = index(j, i);
                        if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
                    }
                }
                for(int j = rows; j >= 0; j--)
                    for(int i = std::max(j - s * 2, 0); i <= j - 1; i++)
                    {
                        int idx = index(j, i);
                        if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
                    }
                rows = n - !end;
                for(int i = 0; i < rows; i++)
                    dp.col(i) = dc.row(s * (i + 1) * 2 - 1).transpose();
                
                /* Propogate gradient to times */
                dt.resize(n);
                int offset = 0;
                const int od = order();
                Eigen::Matrix<double, -1, dim> m = 
                Eigen::MatrixXd::Zero(s << 1, dim);
                for(int p = 0; p < n - 1; offset = (m.setZero(), 2 * s * ++p))
                {
                    for(int i = 1, r = od; i < od; r--, i++)
                    {
                        int idx = (i + s - 1) % (s << 1);
                        for(int j = 0; j < r; j++)
                            m.row(idx) -= times(p, j)
                                        * factorial(i + j, j) 
                                        * b.row(offset + i + j);
                    }
                    m.row(s - 1) = m.row(s);
                    dt[p] = m.cwiseProduct(
                        dc.middleRows(offset + s, m.rows())
                    ).sum();
                }
                for(int i = 1, r = od; i <= s; r--, i++)
                    for(int j = 0; j < r; j++)
                        m.row(i - 1) -= times(n - 1, j)
                                      * factorial(i + j, j)
                                      * b.row(offset + i + j);
                dt[n - 1] = m.topRows(s).cwiseProduct(
                    dc.middleRows(s * (n * 2 - 1), s)
                ).sum();
                dt += dt0;
            }

            /* Calculate trajectory cost and gradients */
            double cost(Eigen::Matrix<double, -1, dim>& c, 
                        Eigen::VectorXd& t, double lambda) const
            {
                /* Calculate trajectory cost partial gradient by coefficient */
                c = Eigen::MatrixXd::Zero(2 * s * n, dim);
                for(int p = 0; p < n; p++)
                {
                    int idx = s * (p * 2 + 1);
                    for(int i = 0; i < s; i++)
                        for(int j = 0; j < s; j++)
                            c.row(idx + i) += 2 * lambda * q(i, j) 
                                            * times(p, i + j + 1)
                                            * b.row(idx + j);
                }
                
                /* Calculate trajectory cost and its partial gradient by time */
                double e, energy = 0;
                t = Eigen::VectorXd::Zero(n);
                for(int p = 0; p < n; p++)
                {
                    int idx = s * (p * 2 + 1);
                    for(int i = 0; i < s; i++)
                        for(int j = i, k = i; j >= 0; j--)
                        {
                            e = lambda
                              * q(i, i - j) * (j? 2: 1)
                              * b.row(idx + i).dot(b.row(idx + i - j)); 
                            t[p] += (k + 1) * e * times(p, k);
                            energy += e * times(p, ++k);
                        }
                }
                return energy;
            }

            /* Diffeomorphic transformation of time */
            void diffeomorphism(const Eigen::VectorXd& input, 
                                Eigen::VectorXd& output,
                                bool forward) const
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

            /* Get coefficient matrix */
            const Eigen::Matrix<double, -1, dim>& coefficients()
            const {return b;}

            /* Get dimension */
            constexpr int dimension() const {return dim;}

            /* Get order = 2s - 1 */
            const int order() const {return s * 2 - 1;}

            /* Get the number of pieces */
            int pieces() const {return n;}

        private:
            int index(int i, int j) const
            {
                return 2 * (2 * s + i - j) * s * n + j;
            }
    };
}
