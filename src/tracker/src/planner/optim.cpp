/* @Author YueLin */

#include "planner/optim.hpp"

namespace eva_tracker
{
    Optimizer::Optimizer(int k, Minco<4>* m, Bezier* b, 
                         ESDF* rc, ESDF* cam, double tau,
                         double vh, double vv, double va,
                         double ah, double av, double aa,
                         double lp, double lo, double lv,
                         double la, double gm, double lw,
                         double ep, double del, double e,
                         double a, double w, double step,
                         int memory, int pst, int iteration):
        bezier(b), minco(m),
        robot(rc), fov(cam), 
        kappa(k), duration(tau),
        lambda{lo * lw, lv * lw, la * lw, lw, lp * lw, gm * lw},
        v2(vh * vh, vv * vv, va * va), a2(ah * ah, av * av, aa * aa),
        mem(memory), past(pst), iterations(iteration),
        epsilon(ep), delta(del), eps(e), armijo(a), wolfe(w), steps(step)
    {
        costs.resize(Nc);
    }

    double Optimizer::optimize(Trajectory* trajectory)
    {
        /* Prepare variables */
        const int n = minco->pieces() * (minco->dimension() + 1);
        Eigen::VectorXd x(n), g(n), x0(n), g0(n);

        /* Initialize variable */
        iters = 0;
        forward(x, tau, pts);

        /* L-BFGS optimization */
        double fx = f(x, g); 
        if(g.cwiseAbs().maxCoeff() / std::max(x.cwiseAbs().maxCoeff(), 1.) > 0)
        {
            /* Initialize the limited memory */
            Eigen::VectorXd d = -g;
            Eigen::MatrixXd s = Eigen::MatrixXd::Zero(n, mem);
            Eigen::MatrixXd y = Eigen::MatrixXd::Zero(n, mem);
            Eigen::VectorXd limit = Eigen::VectorXd::Zero(mem);
            Eigen::VectorXd memory = Eigen::VectorXd::Zero(mem);

            /* Initialize variables */
            int iter = 1, end = 0, bound = 0;
            std::vector<double> pf(past);
            double step = 1. / d.norm();
            pf[0] = fx;

            /* Main loop */
            while(true)
            {
                /* Store the current position and gradient vectors */
                x0 = x; g0 = g;

                /* Adjust the step */
                if(step >= steps)
                    step = steps * 0.5;

                /* Compute the initial gradient in the line search direction */
                bool success = false;
                double grad = g0.dot(d);
                if(grad <= 0)
                {
                    /* Lewis-Overton line search */
                    int iteration = 0;
                    const double w = wolfe * grad;
                    const double a = armijo * grad;
                    bool ac = false, touched = false;
                    double f0 = fx, max = steps, min = 0;
                    while(true)
                    {
                        /* Evaluate the function and gradient values */
                        fx = f(x = x0 + step * d, g);
                        if(std::isinf(fx) || std::isnan(fx))
                            break;
                        
                        /* Check the Armijo condition */
                        if(step * a < fx - f0)
                        {
                            ac = true;
                            max = step;
                        }

                        /* Check the waek Wolfe condition */
                        else if(w > g.dot(d))
                            min = step;

                        /* Line search is successful */
                        else
                        {
                            success = true;
                            break;
                        }
                        
                        /* Maximum number of iteration */
                        if(++iteration >= iterations)
                            break;

                        /* Relative interval width is at least machine precision */
                        if(ac && (max - min) < max * 1e-16)
                            break;

                        /* Update step */
                        if(ac) step = (min + max) / 2; else step *= 2;
                        if(step < 1. / steps) break;
                        if(step > steps)
                        {
                            if(touched) break;
                            touched = true;
                            step = steps;
                        }
                    }
                }
                if(!success)
                {
                    x = x0; g = g0; break;
                }

                /* Convergance test */
                if(g.cwiseAbs().maxCoeff() / 
                   std::max(x.cwiseAbs().maxCoeff(), 1.) <= epsilon)
                    break;

                /* Test for stopping criterion */
                if(iter >= past && 
                    std::fabs(pf[iter % past] - fx) / 
                    std::max(std::fabs(fx), 1.) < delta) 
                    break;
                pf[iter++ % past] = fx;

                /* L-BFGS update */
                d = -g;
                s.col(end) = x - x0;
                y.col(end) = g - g0;

                /* Cautious update */
                double yty = y.col(end).squaredNorm();
                double yts = memory[end] = y.col(end).dot(s.col(end));
                if(yts > eps * s.col(end).squaredNorm() * g0.norm())
                {
                    int j = end = (end + 1) % mem;
                    bound = std::min(mem, bound + 1);
                    for(int _ = bound; _; --_)
                    {
                        j = (j + mem - 1) % mem;
                        limit[j] = s.col(j).dot(d) / memory[j];
                        d -= limit[j] * y.col(j);
                    }
                    d *= yts / yty;
                    for(int _ = bound; _; --_)
                    {
                        d += s.col(j)
                            * (limit[j] - y.col(j).dot(d) / memory[j]);
                        j = (j + 1) % mem;
                    }
                }
                step = 1;
            }
        }

        /* Get the optimal points and times */
        backward(x);
        minco->diffeomorphism(tau, times, 1);

        ROS_DEBUG("Planning completed.");
        ROS_DEBUG("Jo: %f", costs[Jo]);
        ROS_DEBUG("Jd: %f", costs[Jv]);
        ROS_DEBUG("Ja: %f", costs[Ja]);
        ROS_DEBUG("Je: %f", costs[Je]);
        ROS_DEBUG("Jt: %f", costs[Jt]);
        ROS_DEBUG("Jp: %f", costs[Jp]);
        ROS_DEBUG("Total cost: %f", fx);
        ROS_DEBUG("Iters: %d\n", iterations);

        /* Set the optimal parameters of MINCO */
        minco->set(endpoint);
        minco->set(times, pts);
        minco->get(trajectory);
        return fx;
    }

    bool Optimizer::setup(const Eigen::Matrix4Xd& path, Eigen::Matrix4Xd* se,
                          pcl::PointCloud<pcl::PointXYZ>* world)
    {
        int n = path.cols();
        if(--n <= 0)
            return false;
        cloud = world;

        /* Initialize start and end points */
        Eigen::Matrix4Xd* start = se; start->col(0) = path.col(0);
        Eigen::Matrix4Xd* end = se + 1; end->col(0) = path.col(n);
        
        /* Initialize points and times */
        pts.resize(minco->dimension(), n - 1);
        for(int p = 1; p < n; p++)
            pts.col(p - 1) = path.col(p);
        times = Eigen::VectorXd::Constant(n, duration);

        /* Initialize gradients */
        dt.resize(n);
        dv.resize(n);
        dp.resize(minco->dimension(), n);
        dc.resize(n-- * (minco->order() + 1) * 2, minco->dimension());

        /* Adjust yaw angle */
        const int yaw = minco->dimension() - 1;
        double theta = path(yaw, 0);
        for(int p = 0; p < n; p++)
        {
            if(std::fabs(pts(yaw, p) - theta) > PI)
                pts(yaw, p) += 2 * PI * std::floor(
                    (theta - pts(yaw, p)) / (PI * 2) + 0.5
                );
            theta = pts(yaw, p);
        }
        if(std::fabs(end->coeff(yaw, 0) - theta) > PI)
            end->coeffRef(yaw, 0) += 2 * PI * std::floor(
                (theta - end->coeff(yaw, 0)) / (PI * 2) + 0.5
            );
        
        /* Initialize MINCO trajectory */
        endpoint = end->col(0);
        minco->initialize(n + 1, se);
        minco->diffeomorphism(times, tau, 0);
        return true;
    }

    double Optimizer::lbfgs(Eigen::VectorXd& x)
    {
        /* Prepare intermediate variables */
        const int n = x.size();
        Eigen::VectorXd g(n), x0(n), g0(n);

        /* Initialize the limited memory */
        std::vector<double> pf(past);
        Eigen::MatrixXd s = Eigen::MatrixXd::Zero(n, mem);
        Eigen::MatrixXd y = Eigen::MatrixXd::Zero(n, mem);
        Eigen::VectorXd limit = Eigen::VectorXd::Zero(mem);
        Eigen::VectorXd memory = Eigen::VectorXd::Zero(mem);

        /* Evaluate the function value and its gradient */
        double fx = pf[0] = f(x, g); 
        Eigen::VectorXd d = -g;

        /* Main loop */
        if(g.cwiseAbs().maxCoeff() / std::max(x.cwiseAbs().maxCoeff(), 1.) > 0)
        {
            int iter = 1, end = 0, bound = 0;
            double step = 1. / d.norm();
            while(true)
            {
                /* Store the current position and gradient vectors */
                x0 = x; g0 = g;

                /* Adjust the step */
                if(step >= steps)
                    step = steps * 0.5;

                /* Compute the initial gradient in the line search direction */
                bool success = false;
                double grad = g0.dot(d);
                if(grad <= 0)
                {
                    /* Lewis-Overton line search */
                    int iteration = 0;
                    const double w = wolfe * grad;
                    const double a = armijo * grad;
                    bool ac = false, touched = false;
                    double f0 = fx, max = steps, min = 0;
                    while(true)
                    {
                        /* Evaluate the function and gradient values */
                        fx = f(x = x0 + step * d, g);
                        if(std::isinf(fx) || std::isnan(fx))
                            break;
                        
                        /* Check the Armijo condition */
                        if(step * a < fx - f0)
                        {
                            ac = true;
                            max = step;
                        }

                        /* Check the waek Wolfe condition */
                        else if(w > g.dot(d))
                            min = step;

                        /* Line search is successful */
                        else
                        {
                            success = true;
                            break;
                        }
                        
                        /* Maximum number of iteration */
                        if(++iteration >= iterations)
                            break;

                        /* Relative interval width is at least machine precision */
                        if(ac && (max - min) < max * 1e-16)
                            break;

                        /* Update step */
                        if(ac) step = (min + max) / 2; else step *= 2;
                        if(step < 1. / steps) break;
                        if(step > steps)
                        {
                            if(touched) break;
                            touched = true;
                            step = steps;
                        }
                    }
                }
                if(!success)
                {
                    x = x0; g = g0; break;
                }

                /* Convergance test */
                if(g.cwiseAbs().maxCoeff() / 
                   std::max(x.cwiseAbs().maxCoeff(), 1.) <= epsilon)
                    break;

                /* Test for stopping criterion */
                if(iter >= past && 
                    std::fabs(pf[iter % past] - fx) / 
                    std::max(std::fabs(fx), 1.) < delta) 
                    break;
                pf[iter++ % past] = fx;

                /* L-BFGS update */
                d = -g;
                s.col(end) = x - x0;
                y.col(end) = g - g0;

                /* Cautious update */
                double yty = y.col(end).squaredNorm();
                double yts = memory[end] = y.col(end).dot(s.col(end));
                if(yts > eps * s.col(end).squaredNorm() * g0.norm())
                {
                    int j = end = (end + 1) % mem;
                    bound = std::min(mem, bound + 1);
                    for(int _ = bound; _; --_)
                    {
                        j = (j + mem - 1) % mem;
                        limit[j] = s.col(j).dot(d) / memory[j];
                        d -= limit[j] * y.col(j);
                    }
                    d *= yts / yty;
                    for(int _ = bound; _; --_)
                    {
                        d += s.col(j)
                            * (limit[j] - y.col(j).dot(d) / memory[j]);
                        j = (j + 1) % mem;
                    }
                }
                step = 1;
            }
        }
        return fx;
    }

    void Optimizer::forward(Eigen::VectorXd& data,
                            const Eigen::VectorXd& vector,
                            const Eigen::Matrix4Xd& matrix)
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

    void Optimizer::backward(const Eigen::VectorXd& vector)
    {
        const int n = minco->pieces();
        tau = vector.head(n);
        for(int axis = 0; axis < 4; axis++)
        {
            pts.row(axis) = vector.segment(n * (axis + 1), n - 1).transpose();
            endpoint[axis] = vector[n * (axis + 2) - 1];
        }
    }

    double Optimizer::f(const Eigen::VectorXd& var, Eigen::VectorXd& gradients)
    {
        /* Constants */
        const double k = 1. / kappa;
        const int s = minco->order() + 1;
        
        /* Backward variables to points and tau */
        backward(var);
        minco->diffeomorphism(tau, times, 1);

        /* Initialize MINCO trajectory */
        minco->set(endpoint);
        minco->set(times, pts);

        /* Initialize variables */
        costs.setZero();
        Eigen::Vector4d grads;
        Eigen::Matrix4d states;
        int n = minco->pieces();
        Eigen::Matrix3d rotate, dr;
        std::vector<double> t(s, 1);
        double g, p, w, sin, cos, ts = 0;
        Eigen::Vector3d point, target, transform, grad;
        Eigen::MatrixX4d beta = Eigen::MatrixX4d::Zero(s, 4);

        /* Calculate trajectory cost */
        costs[Je] = minco->cost(dc, dt, lambda[Je]);

        /* Calculate visibility cost and penalties */
        for(int i = 0; i < n; i++)
        {
            if(i) ts += times[i - 1];
            for(int j = 0; j <= kappa; j++)
            {
                for(int order = 1; order < s; order++)
                    t[order] = t[order - 1] * j * k * times[i];
                for(int order = 0; order < 4; order++)
                {
                    for(int div = 0, od = order; od < s; div++, od++)
                        beta(od, order) = t[div] * factorial(od, div);
                    states.col(order) = minco->coefficients().middleRows(
                        i * s, s
                    ).transpose() * beta.col(order);
                }
                sin = std::sin(states.col(0).w());
                cos = std::cos(states.col(0).w());
                w = k * (!j || j == kappa? 0.5: 1);
                dr << -sin, cos, 0, -cos, -sin, 0, 0, 0, 0;
                rotate << cos, sin, 0, -sin, cos, 0, 0, 0, 1;

                /* Observation penalty */
                g = w * lambda[Jv];
                target = (*bezier)[ts + t[1]] - states.col(0).head(3);
                p = fov->max() - fov->value(rotate * target, &grad);
                costs[Jv] += 0.5 * g * p * p * times[i];
                dc.block(i * s, 0, s, 3) += g * times[i]
                                          * p * beta.col(0)
                                          * grad.transpose() * rotate;
                transform = dr * target;
                dc.block(i * s, 3, s, 1) -= g * times[i]
                                          * p * beta.col(0)
                                          * transform.transpose() * grad;
                point = bezier->derivative(ts + t[1]) - states.col(1).head(3);
                dt[i] += g * p * (
                    0.5 * p - t[1] * (transform + rotate * point).dot(grad)
                );

                /* Angle penalty */
                sin = states.col(0).w() - std::atan2(target.y(), target.x());
                g = w * lambda[Ja];
                p = 1 - std::cos(sin);
                grad.x() = std::sin(sin);
                costs[Ja] += g * p * times[i];
                grad.y() = 1. / target.head(2).squaredNorm();
                const Eigen::MatrixX3d d = g * times[i] 
                                         * grad.x() * grad.y()
                                         * beta.col(0) * target.transpose(); 
                dc.block(i * s, 0, s, 1) -= d.col(1);
                dc.block(i * s, 1, s, 1) += d.col(0);
                dc.block(i * s, 3, s, 1) += g * times[i] 
                                          * grad.x() * beta.col(0);
                dt[i] += g * (p + t[1] * grad.x() * (states.col(1).w() - (
                    target.x() * point.y() - target.y() * point.x()
                ) * grad.y()));

                /* Occlusion penalty and safety penalty */
                point = states.col(0).head(3);
                for(int pc = 0; pc <= 1; pc++)
                {
                    sin = cos = 0;
                    grads.setZero();
                    Costs j = pc? Jp: Jo;
                    ESDF* esdf = pc? robot: fov;
                    for(pcl::PointXYZ& obstacle: *cloud)
                    {
                        target = Eigen::Vector3d(
                            obstacle.x, obstacle.y, obstacle.z
                        ) - point;
                        if((p = esdf->value(rotate * target, &grad)) > 0)
                        {
                            sin += p;
                            transform = dr * target;
                            grads.head(3) -= rotate.transpose() * grad;
                            grads.w() += grad.dot(transform);
                            cos += grad.dot(
                                transform - rotate * states.col(1).head(3)
                            );
                        }
                    }
                    g = w * lambda[j];
                    dc.block(i * s, 0, s, 3) += g * times[i]
                                              * sin * beta.col(0)
                                              * grads.head(3).transpose();
                    dc.block(i * s, 3, s, 1) += g * sin 
                                              * times[i] 
                                              * grads.w()
                                              * beta.col(0);
                    dt[i] += g * sin * (0.5 * sin + t[1] * cos);
                    costs[j] += 0.5 * g * sin * sin * times[i];
                }

                /* Horizontal velocity penalty */
                point = states.col(1).head(3);
                if((p = point.head(2).squaredNorm() - v2.x()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 0, s, 2) += 2 * g 
                                              * times[i]
                                              * beta.col(1) 
                                              * point.head(2).transpose();
                    dt[i] += g * (
                        2 * t[1] * point.head(2).dot(states.col(2).head(2)) + p
                    );
                }

                /* Vertical velocity penalty */
                if((p = point.z() * point.z() - v2.y()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 2, s, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(1);
                    dt[i] += g * (2 * t[1] * point.z() * states.col(2).z() + p);
                }

                /* Angular velocity penalty */
                point.z() = states.col(1).w();
                if((p = point.z() * point.z() - v2.z()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 3, s, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(1);
                    dt[i] += g * (2 * t[1] * point.z() * states.col(2).w() + p);
                }

                /* Horizontal acceleration penalty */
                point = states.col(2).head(3);
                if((p = point.head(2).squaredNorm() - a2.x()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 0, s, 2) += 2 * g 
                                              * times[i]
                                              * beta.col(2)
                                              * point.head(2).transpose();
                    dt[i] += g * (
                        2 * t[1] * point.head(2).dot(states.col(3).head(2)) + p
                    );
                }

                /* Vertical acceleration penalty */
                if((p = point.z() * point.z() - a2.y()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 2, s, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(2);
                    dt[i] += g * (2 * t[1] * point.z() * states.col(3).z() + p);
                }

                /* Angular acceleration penalty */
                point.z() = states.col(2).w();
                if((p = point.z() * point.z() - a2.z()) > 0)
                {
                    costs[Jp] += g * p * times[i];
                    dc.block(i * s, 3, s, 1) += 2 * g 
                                              * times[i] 
                                              * point.z()
                                              * beta.col(2);
                    dt[i] += g * (2 * t[1] * point.z() * states.col(3).w() + p);
                }
            }
        }

        /* Calculate time regularization term */
        ts = n * duration - times.sum();
        while(n--)
            dt[n] -= 2 * lambda[Jt] * ts;
        costs[Jt] = lambda[Jt] * ts * ts;

        /* Propagate gradients to points and tau */
        minco->propogate(dc, dt, dp, dv, true);
        minco->propogate(tau, dv, dv);

        /* Forward gradients to dv and dp */
        forward(gradients, dv, dp);

        /* Sum all costs */
        return ++iters, costs.sum();
    }
}
