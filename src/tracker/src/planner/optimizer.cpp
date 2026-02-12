/* @Author YueLin */

#include "planner/optim.hpp"

namespace eva_tracker
{
    Optimizer::Optimizer(int k, int n, Minco* path, Bezier* b, 
                         ESDF* rc, ESDF* camera, double tau, 
                         double delta, double step, int mem,
                         double vh, double vv, double va,
                         double ah, double av, double aa,
                         double p, double o, double d,
                         double a, double r, double w):
        kappa(k), duration(tau),
        minco(path), bezier(b),
        robot(rc), fov(camera),
        vh2(vh * vh), vv2(vv * vv), va2(va * va), 
        ah2(ah * ah), av2(av * av), aa2(aa * aa),
        lambda_p(p * w), lambda_o(o * w), lambda_d(d * w), 
        lambda_a(a * w), gamma(r * w), weight(w)
    {
        params.delta = delta;
        params.mem_size = mem;
        params.min_step = step;
        params.g_epsilon = 0.0;
        params.max_iterations = n;
    }

    double Optimizer::optimize(Trajectory* trajectory)
    {
        /* Initialize */
        Eigen::VectorXd var(minco->pieces() * (minco->dim() + 1));
        forward(var, tau, points);

        /* Optimization */
        double cost;
        iterations = 0;
        lbfgs::lbfgs_optimize(var, cost, f, nullptr, nullptr, this, params);

        /* Get the optimal points and times */
        backward(var);
        minco->diffeomorphism(tau, times, true);

        // std::cout << "iters: " << iterations
        //           << "\nJ_o: " << costs[0]
        //           << "\nJ_v: " << costs[1]
        //           << "\nJ_a: " << costs[2]
        //           << "\nJ_e: " << costs[3]
        //           << "\nJ_t: " << costs[4]
        //           << "\nJ_p: " << costs[5] << std::endl;

        /* Set the optimal parameters of MINCO */
        minco->set(endpoint);
        minco->set(points, times);
        minco->get(trajectory);
        return cost;
    }

    bool Optimizer::setup(Eigen::MatrixXd& path,
                          Eigen::MatrixXd* states,
                          pcl::PointCloud<pcl::PointXYZ>* world)
    {
        int n = path.cols();
        if(--n <= 0)
            return false;
        cloud = world;
        
        /* Initialize points and times */
        states[0].col(0) = path.col(0);
        states[1].col(0) = path.col(n);
        points.resize(minco->dim(), n - 1);
        for(int p = 1; p < n; p++)
            points.col(p - 1) = path.col(p);
        times = duration * Eigen::VectorXd::Ones(n);

        /* Initialize gradients */
        dt.resize(n);
        dv.resize(n);
        dp.resize(4, n);
        dc.resize(n-- * (minco->order() + 1) * 2, 4);

        /* Adjust yaw angle */
        int yaw = minco->dim() - 1;
        double theta = path(yaw, 0);
        for(int p = 0; p < n; p++)
        {
            if(std::fabs(points(yaw, p) - theta) > PI)
                points(yaw, p) += 2 * PI * std::floor(
                    (theta - points(yaw, p)) / (PI * 2) + 0.5
                );
            theta = points(yaw, p);
        }
        if(std::fabs(states[1](yaw, 0) - theta) > PI)
            states[1](yaw, 0) += 2 * PI * std::floor(
                (theta - states[1](yaw, 0)) / (PI * 2) + 0.5
            );
        
        /* Initialize MINCO trajectory */
        endpoint = states[1].col(0);
        minco->initialize(states[0], states[1], n + 1);
        minco->diffeomorphism(times, tau, false);
        return true;
    }

    inline void Optimizer::integral()
    {
        /* Constants */
        const double k = 1. / kappa;
        const int N = minco->pieces();
        const int S = minco->order() + 1;
        const Eigen::MatrixXd& b = minco->coefficients();

        /* Variables */
        Eigen::Matrix3d rotate, dr;
        std::vector<double> t(S, 1);
        Eigen::Vector4d states[4], grads;
        double g, p, w, sin, cos, phi, ts = 0;
        Eigen::Vector3d point, target, transform, grad;
        Eigen::MatrixX4d beta = Eigen::MatrixX4d::Zero(S, 4);

        /* Numerical integration */
        for(int i = 0; i < N; i++)
        {
            if(i) ts += times[i - 1];
            const Eigen::MatrixXd& c = b.block(i * S, 0, S, 4).transpose();
            for(int j = 0; j <= kappa; j++)
            {
                for(int s = 1; s < S; s++)
                    t[s] = t[s - 1] * j * k * times[i];
                for(int order = 0; order <= 3; order++)
                {
                    for(int div = 0, s = order; s < S; div++, s++)
                        beta(s, order) = t[div] * minco->div(s, div);
                    states[order] = c * beta.col(order);
                }
                sin = std::sin(states[0].w());
                cos = std::cos(states[0].w());
                w = k * (!j || j == kappa? 0.5: 1);
                dr << -sin, cos, 0, -cos, -sin, 0, 0, 0, 0;
                rotate << cos, sin, 0, -sin, cos, 0, 0, 0, 1;

                /* Observation penalty */
                g = w * lambda_d;
                target = (*bezier)[ts + t[1]] - states[0].head(3);
                point = bezier->derivative(ts + t[1]) - states[1].head(3);
                p = fov->max() - fov->value(rotate * target, &grad);
                costs[1] += 0.5 * g * p * p * times[i];
                dc.block(i * S, 0, S, 3) += g * times[i]
                                          * p * beta.col(0)
                                          * grad.transpose() * rotate;
                transform = dr * target;
                dc.block(i * S, 3, S, 1) -= g * times[i]
                                          * p * beta.col(0)
                                          * transform.transpose() * grad;
                dt[i] += g * p * (0.5 * p - t[1] * (
                    transform + rotate * point
                ).dot(grad));

                /* Angle penalty */
                phi = states[0].w() - std::atan2(target.y(), target.x());
                g = w * lambda_a;
                p = 1 - std::cos(phi);
                grad[0] = std::sin(phi);
                costs[2] += g * p * times[i];
                grad[1] = 1. / target.head(2).squaredNorm();
                Eigen::MatrixX3d d = g * times[i] * grad[0] * grad[1]
                                   * beta.col(0) * target.transpose(); 
                dc.block(i * S, 0, S, 1) -= d.col(1);
                dc.block(i * S, 1, S, 1) += d.col(0);
                dc.block(i * S, 3, S, 1) += g * times[i] 
                                          * grad[0] * beta.col(0);
                dt[i] += g * (p + t[1] * grad[0] * (states[1][3] - (
                    target.x() * point.y() - target.y() * point.x()
                ) * grad[1]));

                /* Occlusion penalty and safety penalty */
                point = states[0].head(3);
                for(int pc = 0; pc <= 1; pc++)
                {
                    phi = cos = 0;
                    grads.setZero();
                    ESDF* esdf = pc? robot: fov;
                    g = w * (pc? lambda_p: lambda_o);
                    for(pcl::PointXYZ& obstacle: *cloud)
                    {
                        target = Eigen::Vector3d(
                            obstacle.x, obstacle.y, obstacle.z
                        ) - point;
                        if((p = esdf->value(rotate * target, &grad)) > 0)
                        {
                            phi += p;
                            transform = dr * target;
                            grads.head(3) -= rotate.transpose() * grad;
                            grads[3] += grad.dot(transform);
                            cos += grad.dot(
                                transform - rotate * states[1].head(3)
                            );
                        }
                    }
                    dc.block(i * S, 0, S, 3) += g * times[i]
                                              * phi * beta.col(0)
                                              * grads.head(3).transpose();
                    dc.block(i * S, 3, S, 1) += g * phi 
                                              * times[i] 
                                              * grads[3]
                                              * beta.col(0);
                    dt[i] += g * phi * (0.5 * phi + t[1] * cos);
                    costs[pc * 5] += 0.5 * g * phi * phi * times[i];
                }

                /* Horizontal velocity penalty */
                point = states[1].head(3);
                if((p = point.head(2).squaredNorm() - vh2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 0, S, 2) += 2 * g 
                                              * times[i]
                                              * beta.col(1) 
                                              * point.head(2).transpose();
                    dt[i] += g * (
                        2 * t[1] * point.head(2).dot(states[2].head(2)) + p
                    );
                }

                /* Vertical velocity penalty */
                if((p = point.z() * point.z() - vv2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 2, S, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(1);
                    dt[i] += g * (2 * t[1] * point.z() * states[2].z() + p);
                }

                /* Angular velocity penalty */
                point[0] = states[1].w();
                if((p = point[0] * point[0] - va2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 3, S, 1) += 2 * g 
                                              * times[i] 
                                              * point[0]
                                              * beta.col(1);
                    dt[i] += g * (2 * t[1] * point[0] * states[2].w() + p);
                }

                /* Horizontal acceleration penalty */
                point = states[2].head(3);
                if((p = point.head(2).squaredNorm() - ah2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 0, S, 2) += 2 * g 
                                              * times[i]
                                              * beta.col(2)
                                              * point.head(2).transpose();
                    dt[i] += g * (
                        2 * t[1] * point.head(2).dot(states[3].head(2)) + p
                    );
                }

                /* Vertical acceleration penalty */
                if((p = point.z() * point.z() - av2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 2, S, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(2);
                    dt[i] += g * (2 * t[1] * point.z() * states[3].z() + p);
                }

                /* Angular acceleration penalty */
                point[0] = states[2].w();
                if((p = point[0] * point[0] - aa2) > 0)
                {
                    costs[5] += g * p * times[i];
                    dc.block(i * S, 3, S, 1) += 2 * g 
                                              * times[i] 
                                              * point[0]
                                              * beta.col(2);
                    dt[i] += g * (2 * t[1] * point[0] * states[3].w() + p);
                }
            }
        }
    }

    inline void Optimizer::regularization()
    {
        int n = minco->pieces();
        double ts = n * duration - times.sum();
        while(n--) dt[n] -= 2 * gamma * ts;
        costs[4] = gamma * ts * ts;
    }

    inline void Optimizer::forward(Eigen::VectorXd& data,
                                   Eigen::VectorXd& vector,
                                   Eigen::MatrixXd& matrix)
    {
        const int n = minco->pieces();
        data.head(n) = vector;
        if(n == matrix.cols()) for(int axis = 0; axis < 4; axis++)
            data.segment(n * (axis + 1), n) = matrix.row(axis).transpose();
        else for(int axis = 0; axis < 4; axis++)
        {
            data.segment(n * (axis + 1), n - 1) = matrix.row(axis).transpose();
            data[n * (axis + 1) + n - 1] = endpoint[axis];
        }
    }

    inline void Optimizer::backward(const Eigen::VectorXd& x)
    {
        const int n = minco->pieces();
        tau = x.head(n);
        for(int axis = 0; axis < 4; axis++)
        {
            points.row(axis) = x.segment(n * (axis + 1), n - 1).transpose();
            endpoint[axis] = x[n * (axis + 2) - 1];
        }
    }

    double Optimizer::f(void* optimizer,
                        const Eigen::VectorXd& var,
                        Eigen::VectorXd& gradients)
    {
        /* Initialize optimizer */
        Optimizer* self = reinterpret_cast<Optimizer*>(optimizer);
        self->costs = Eigen::VectorXd::Zero(6);
        
        /* Backward variables to points and tau */
        self->backward(var);
        self->minco->diffeomorphism(self->tau, self->times, true);

        /* Initialize MINCO trajectory */
        self->minco->set(self->endpoint);
        self->minco->set(self->points, self->times);

        /* Calculate trajectory cost */
        self->costs[3] = self->minco->cost(self->dc, self->dt, self->weight);

        /* Calculate time regularization term */
        self->regularization();

        /* Calculate visibility cost and penalties */
        self->integral();

        /* Propagate gradients to points and tau */
        self->minco->propogate(self->dc, self->dt, self->dp, self->dv, true);
        self->minco->propogate(self->tau, self->dv, self->dv);

        /* Forward gradients to dv and dp */
        self->forward(gradients, self->dv, self->dp);

        /* Sum all costs */
        ++self->iterations;
        return self->costs.sum();
    }
}
