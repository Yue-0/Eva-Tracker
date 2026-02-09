/* @Author: YueLin */

#include <queue>
#include <limits>
#include <algorithm>

#include <Eigen/Eigen>

#include "geometry_msgs/PoseStamped.h"

#include "simulator/planner.hpp"

namespace simulator
{
    Planner::~Planner()
    {
        delete[] g;
        delete[] parent;
        delete[] visited;
    }

    Planner::Planner(Map* world, Robot* robot,
                     double dt, double vel, double acc,
                     double lmd, int pst, int m, int itr,
                     double e, double step, double del,
                     double ep, double wol, double arm):
        map(world), car(robot), time(dt), ds(0.5 * std::sqrt(
            std::pow(robot->length, 2) + std::pow(robot->width, 2)
        )), vm(vel), am(acc), lambda(lmd), past(pst), mem(m), iterations(itr),
        eps(e), steps(step), delta(del), epsilon(ep), wolfe(wol), armijo(arm)
    {
        /* For A-Star */
        int size = world->size.x() * world->size.y();
        visited = new bool[size];
        parent = new int[size];
        g = new double[size];

        /* For B-Spline */
        cp << 1, 4, 1; cp /= 6;
        cv << -1, 0, 1; cv /= 2 * dt;
        ca << 1, -2, 1; ca /= dt * dt;

        /* For L-BFGS */
        pf = Eigen::VectorXd::Zero(pst);
        limit = Eigen::VectorXd::Zero(m);
        memory = Eigen::VectorXd::Zero(m);
    }

    nav_msgs::Path Planner::msg(std::string& frame,
                                std::vector<Eigen::Vector2d>& ctrl)
    {
        /* Initialize message */
        nav_msgs::Path message;
        const int n = ctrl.size() - 2;
        message.header.frame_id = frame;
        if(n <= 1) return message; 

        /* Convert control points to path */
        for(int p = 0; p < n; p++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = message.header.frame_id;
            pose.pose.position.x = (
                ctrl[p].x() + 4 * ctrl[p + 1].x() + ctrl[p + 2].x()
            ) / 6.;
            pose.pose.position.y = (
                ctrl[p].y() + 4 * ctrl[p + 1].y() + ctrl[p + 2].y()
            ) / 6.;
            message.poses.push_back(pose);
        }
        return message;
    }

    std::vector<Eigen::Vector2d> Planner::plan(double xg, double yg)
    {
        /* A-Star search */
        const double r = 1. / map->resolution;
        std::vector<Eigen::Vector2d> points = astar(
            std::round(r * car->pose.x()),
            std::round(r * car->pose.y()),
            std::round(r * xg), std::round(r * yg)
        );
        int n = points.size() - 1;
        if(n <= 1)
            return points;
        
        /* Sampling */
        int step = std::round(vm * time * r);
        std::vector<Eigen::Vector2d> path;
        for(int p = 0; p < n; p += step)
            path.push_back(points[p]);
        path.push_back(points[n]);

        /* B-Spline optimization */
        Eigen::Matrix2Xd bsp = bspline(path);
        if(bsp.cols() > 6)
            optimize(bsp);
        path.resize(bsp.cols());
        Eigen::Map<Eigen::Matrix2Xd>(path.front().data(), 2, bsp.cols()) = bsp;
        return path;
    }

    Eigen::Vector4d Planner::control(std::vector<Eigen::Vector2d>& path)
    {
        /* Initialize */
        Eigen::Vector4d ctrl = Eigen::Vector4d::Zero();
        const int n = path.size() - 1;
        if(n <= 1) return ctrl;

        /* Get next waypoint */
        Eigen::Vector3d point;
        point.x() = (path[n].x() + 4 * path[n - 1].x() + path[n - 2].x()) / 6.;
        point.y() = (path[n].y() + 4 * path[n - 1].y() + path[n - 2].y()) / 6.;
        point.z() = std::atan2(
            point.y() - car->pose.y(), point.x() - car->pose.x()
        );
        path.pop_back();

        /* Calculate velocity */
        ctrl.head(2) = (point.head(2) - car->pose.head(2)) / time;
        ctrl.w() = clip(point.z() - car->pose.w());
        if(ctrl.w() < -PI / 2) ctrl.w() += PI;
        if(ctrl.w() > PI / 2) ctrl.w() -= PI;
        ctrl.w() /= time;
        return ctrl;
    }

    void Planner::bfs(int* xs, int* ys)
    {
        /* Variables */
        int x0 = *xs, y0 = *ys;
        int index = encode(x0, y0);

        /* Initialize arrays */
        std::fill_n(
            g, map->size.x() * map->size.y(), map->size.x() * map->size.y() * 2
        );
        std::fill_n(visited, map->size.x() * map->size.y(), false);
        visited[index] = true;
        g[index] = 0;

        /* Push the first point into the queue */
        std::queue<int> queue;
        queue.push(index);

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            index = queue.front(); queue.pop();

            /* If the target is found */
            if(g[index] > 1)
            {
                decode(index, xs, ys);
                return;
            }
            
            /* Expand the point */
            decode(index, &x0, &y0);
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = x0 + dx;
                if(x >= 0 && x < map->size.x())
                    for(int dy = -1; dy <= 1; dy++)
                    {
                        int y = y0 + dy;
                        if(y >= 0 && y < map->size.y())
                        {
                            int idx = encode(x, y);
                            if(!visited[idx])
                            {
                                queue.push(idx);
                                visited[idx] = true;
                                g[idx] = map->exp[x][y][0]? 0: g[idx] + 1;
                            }
                        }   
                    }   
            }   
        }
    }

    std::vector<Eigen::Vector2d> Planner::astar(int xs, int ys, int xg, int yg)
    {
        /* Boundaries */
        xs = std::max(std::min(xs, map->size.x() - 1), 0);
        ys = std::max(std::min(ys, map->size.y() - 1), 0);
        xg = std::max(std::min(xg, map->size.x() - 1), 0);
        yg = std::max(std::min(yg, map->size.y() - 1), 0);

        /* Result */
        std::vector<Eigen::Vector2d> path;

        /* Obstacle check */
        if(map->exp[xg][yg][0])
            bfs(&xg, &yg);
        if(map->exp[xs][ys][0])
            bfs(&xs, &ys);

        /* Initialize arrays */
        int index = map->size.x() * map->size.y();
        std::fill_n(g, index, std::numeric_limits<double>::infinity());
        std::fill_n(visited, index, false);
        std::fill_n(parent, index, -1);
        g[index = encode(xs, ys)] = 0;

        /* Push the first point into the queue */
        std::priority_queue<std::pair<double, int>> queue;
        queue.push(std::make_pair(-f(g[index], xs, ys, xg, yg), index));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            index = queue.top().second;
            decode(index, &xs, &ys);
            queue.pop();

            /* Check visited */
            if(visited[index])
                continue;
            visited[index] = true; 

            /* If found a path */
            if(xs == xg && ys == yg)
            {
                do
                {
                    decode(index, &xs, &ys);
                    path.emplace_back(
                        xs * map->resolution, ys * map->resolution
                    );
                }
                while((index = parent[index]) != -1);
                std::reverse(path.begin(), path.end());
                break;
            }

            /* Expand the point */
            for(int neighbor = 0; neighbor < 9; neighbor++)
            {
                int x = xs + neighbor % 3 - 1;
                int y = ys + neighbor / 3 - 1;
                int idx = encode(x, y);
                
                /* Determine the legitimacy of the extension point */
                if(x < 0 || x >= map->size.x() || y < 0 || y >= map->size.y() || 
                   map->exp[x][y][0] || visited[idx])
                    continue;
                
                /* Calculate cost value */
                double g0 = f(g[index], x, y, xs, ys);

                /* Update the point */
                if(g0 < g[idx])
                {
                    g[idx] = g0;
                    parent[idx] = index;
                    queue.push(std::make_pair(-f(g0, x, y, xg, yg), idx));
                }
            }
        }

        return path;
    }

    Eigen::Matrix2Xd Planner::bspline(const std::vector<Eigen::Vector2d>& path)
    {
        const int n = path.size();
        Eigen::MatrixX2d points(n + 4, 2);
        points.topRows(n) = Eigen::Map<
            const Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>
        >(reinterpret_cast<const double*>(path.data()), n, 2);
        points.bottomRows(4).setZero();
        
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(n + 4, n + 2);
        for(int i = 0; i < 3; i++)
            matrix.diagonal(i).head(n).setConstant(cp[i]);
        matrix.block(n, 0, 1, 3) = cv.transpose();
        matrix.block(n + 2, 0, 1, 3) = ca.transpose();
        matrix.block(n + 1, n - 1, 1, 3) = cv.transpose();
        matrix.block(n + 3, n - 1, 1, 3) = ca.transpose();
        return matrix.colPivHouseholderQr().solve(points).transpose();
    }

    double Planner::optimize(Eigen::Matrix2Xd& ctrl)
    {
        /* Prepare intermediate variables */
        const int n = ctrl.size();
        Eigen::VectorXd grad(n), x0(n), g0(n);
        Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(ctrl.data(), n);

        /* Initialize the limited memory */
        limit.setZero(); memory.setZero();
        Eigen::MatrixXd s = Eigen::MatrixXd::Zero(n, mem);
        Eigen::MatrixXd y = Eigen::MatrixXd::Zero(n, mem);

        /* Evaluate the function value and its gradient */
        double fx = pf[0] = cost(x, grad); 
        Eigen::VectorXd d = -grad;

        if(!convergance(x, grad))
        {
            int iter = 1, end = 0, bound = 0;
            double step = 1. / d.norm();
            while(true)
            {
                /* Store the current position and gradient vectors */
                x0 = x; g0 = grad;

                /* Lewis-Overton line search */
                if(step >= steps) step = steps * 0.5;
                if(!search(grad, &step, x, &fx, d, x0, g0))
                {
                    x = x0; grad = g0; break;
                }

                /* Convergance test */
                if(convergance(x, grad))
                    break;

                /* Test for stopping criterion */
                if(iter >= past && 
                   std::fabs(pf[iter % past] - fx) / 
                   std::max(std::fabs(fx), 1.) < delta) break;
                pf[iter++ % past] = fx;

                /* L-BFGS update */
                d = -grad;
                s.col(end) = x - x0;
                y.col(end) = grad - g0;

                /* Cautious update */
                double yty = y.col(end).squaredNorm();
                double yts = memory[end] = y.col(end).dot(s.col(end));
                if(yts > eps * s.col(end).squaredNorm() * g0.norm())
                {
                    int _;
                    int j = end = (end + 1) % mem;
                    bound = std::min(mem, bound + 1);
                    for(_ = bound; _; --_)
                    {
                        j = (j + mem - 1) % mem;
                        limit[j] = s.col(j).dot(d) / memory[j];
                        d -= limit[j] * y.col(j);
                    }
                    d *= yts / yty;
                    for(_ = bound; _; --_)
                    {
                        d += (limit[j] - y.col(j).dot(d) / memory[j]) * s.col(j)
                        ;j = (j + 1) % mem;
                    }
                }
                step = 1;
            }
        }
        ctrl = Eigen::Map<Eigen::Matrix2Xd>(x.data(), 2, n >> 1);
        return fx;
    }

    double Planner::cost(const Eigen::VectorXd& var, Eigen::VectorXd& grad)
    {
        /* Initialize */
        grad.setZero();
        Eigen::Vector2d temp;
        int n = var.size() >> 1;
        Eigen::Map<const Eigen::Matrix2Xd> ctrl(var.data(), 2, n);
        Eigen::Map<Eigen::Matrix2Xd> gradient(grad.data(), 2, n);

        /* Calculate velocity and acceleration */
        --n; Eigen::Matrix2Xd vel = ctrl.rightCols(n) - ctrl.leftCols(n);
        --n; Eigen::Matrix2Xd acc = vel.rightCols(n) - vel.leftCols(n);
        
        /* Calculate smoothness cost */
        double value = acc.squaredNorm();
        gradient.leftCols(n) += 2 * acc;
        gradient.rightCols(n) += 2 * acc;
        gradient.middleCols(1, n) -= 4 * acc;

        /* Calculate acceleration cost */
        double t2 = 1 / (time * time);
        double t4 = t2 * t2;
        for(int t = 0; t < n; t++)
            if((*g = acc.col(t).squaredNorm() * t4 - am * am) > 0)
            {
                value += *g;
                temp = acc.col(t) * t4;
                gradient.col(t) += temp;
                gradient.col(t + 2) += temp;
                gradient.col(t + 1) -= 2 * temp;
            }

        /* Calculate velocity cost */
        ++n;
        for(int t = 0; t < n; t++)
        {
            if((*g = vel.col(t).squaredNorm() * ca[0] - vm * vm) > 0)
            {
                value += *g;
                temp = vel.col(t) * t2;
                gradient.col(t) -= temp;
                gradient.col(t + 1) += temp;
            }
        }

        /* Calculate safety cost */
        n -= 2;
        for(int t = 3; t < n; t++)
        {
            temp = ctrl.col(t) / map->resolution;
            double x = std::max(std::min(temp.x(), map->size.x() - 2.), 0.);
            double y = std::max(std::min(temp.y(), map->size.y() - 2.), 0.);

            /* Linear interpolation */
            int x1 = x, y1 = y;
            int x2 = x1 + 1, y2 = y1 + 1;
            double u = x - x1, v = y - y1;
            double u_ = 1. - u, v_ = 1. - v;
            *g = u * v * map->sdf[x2][y2]
               + u * v_ * map->sdf[x2][y1]
               + u_ * v * map->sdf[x1][y2]
               + u_ * v_ * map->sdf[x1][y1];

            /* Calculate gradient */
            if((*g = ds - *g) > 0)
            {
                value += lambda ** g ** g;
                grad[t * 2] += 2 * lambda ** g * (
                    + v * map->sdf[x1][y2] + v_ * map->sdf[x1][y1]
                    - v * map->sdf[x2][y2] - v_ * map->sdf[x2][y1]
                );
                grad[t * 2 + 1] += 2 * lambda ** g * (
                    + u * map->sdf[x2][y1] + u_ * map->sdf[x1][y1]
                    - u * map->sdf[x2][y2] - u_ * map->sdf[x1][y2]
                );
            }
        }

        grad.head(6).setZero();
        grad.tail(6).setZero();
        return value;
    }

    bool Planner::search(Eigen::VectorXd& gradient, double* step,
                         Eigen::VectorXd& x, double* value,
                         const Eigen::VectorXd& direction,
                         const Eigen::VectorXd& x0,
                         const Eigen::VectorXd& g0)
    {
        int iter = 0;
        bool ac = false, touched = false;
        double min = 0, max = steps, fx = *value;

        /* Compute the initial gradient in the search direction */
        double grad = g0.dot(direction);
        if(grad > 0) return false;
        const double w = wolfe * grad;
        const double a = armijo * grad;

        /* Line search */
        while(true)
        {
            /* Evaluate the function and gradient values */
            *value = cost(x = x0 + *step * direction, gradient);
            if(std::isinf(*value) || std::isnan(*value))
                return false;
            
            /* Check the Armijo condition */
            if(*step * a < *value - fx)
            {
                max = *step; ac = true;
            }
            /* Check the waek Wolfe condition */
            else if(w > gradient.dot(direction))
                min = *step;
            else
                return true;
            
            /* Maximum number of iteration */
            if(++iter >= iterations) return false;

            /* Relative interval width is at least machine precision */
            if(ac && (max - min) < max * 1e-16) return false;

            /* Update step */
            if(ac) *step = (min + max) / 2; else *step *= 2;
            if(*step < 1. / steps) return false;
            if(*step > steps)
            {
                if(touched) 
                    return false;
                touched = true;
                *step = steps;
            }
        }
    }
}
