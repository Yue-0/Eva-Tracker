/* @Author: YueLin */

#include <queue>
#include <limits>
#include <algorithm>

#include <Eigen/Eigen>

#include "geometry_msgs/PoseStamped.h"

#include "lbfgs"
#include "simulator/planner.hpp"

namespace simulator
{
    Planner::Planner(Map* world, Robot* robot,
                     double dt, double vel, double acc)
    : map(world), car(robot), time(dt), ds(0.5 * std::sqrt(
        std::pow(robot->length, 2) + std::pow(robot->width, 2)
    )), vm(vel), am(acc) {}

    R3xSO2 Planner::control(std::vector<std::pair<double, double>>& path)
    {
        /* Initialize */
        R3xSO2 ctrl = {0.0, 0.0, 0.0, 0.0};
        const int n = path.size() - 1;
        if(n <= 1) return ctrl;

        /* Get next waypoint */
        R3xSO2 point;
        point.y = path[n].second + 4 * path[n - 1].second + path[n - 2].second;
        point.x = path[n].first + 4 * path[n - 1].first + path[n - 2].first;
        point.x /= 6.; point.y /= 6.; path.pop_back();
        point.yaw = std::atan2(point.y - car->pose.y, point.x - car->pose.x);

        /* Calculate velocity */
        ctrl.yaw = clip(point.yaw - car->pose.yaw);
        ctrl.x = (point.x - car->pose.x) / time;
        ctrl.y = (point.y - car->pose.y) / time;
        if(ctrl.yaw < -PI / 2) ctrl.yaw += PI;
        if(ctrl.yaw > PI / 2) ctrl.yaw -= PI;
        ctrl.yaw /= time;
        return ctrl;
    }

    std::vector<std::pair<double, double>> Planner::plan(double xg, double yg)
    {
        const double r = 1. / map->resolution;
        std::vector<std::pair<double, double>> path = astar(
            std::round(r * car->pose.x),
            std::round(r * car->pose.y),
            std::round(r * xg), std::round(r * yg)
        );
        int n = path.size() - 1;
        if(n <= 1) return path;
        int step = std::round(vm * time * r);
        std::vector<std::pair<double, double>> keypoints;
        for(int p = 0; p < n; p += step)
            keypoints.push_back(path[p]);
        keypoints.push_back(path[n]);
        if((keypoints = bspline(keypoints)).size() > 6)
            optimize(keypoints);
        return keypoints;
    }

    nav_msgs::Path Planner::msg(std::string& frame,
                                std::vector<std::pair<double, double>>& ctrl)
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
                ctrl[p].first + 4 * ctrl[p + 1].first + ctrl[p + 2].first
            ) / 6.;
            pose.pose.position.y = (
                ctrl[p].second + 4 * ctrl[p + 1].second + ctrl[p + 2].second
            ) / 6.;
            message.poses.push_back(pose);
        }
        return message;
    }

    void Planner::bfs(int* xs, int* ys)
    {
        /* Variables */
        int x0 = *xs, y0 = *ys;
        std::pair<int, int> xy;

        /* Constants */
        const int H = map->size[X], W = map->size[Y];

        /* Initialize queue and arrays */
        std::queue<std::pair<int, int>> queue;
        std::vector<std::vector<bool>> visited(
            H, std::vector<bool>(W, false)
        );
        std::vector<std::vector<int>> dist(
            H, std::vector<int>(W, H * W * 2)
        );

        /* Push the first point into the queue */
        queue.push(std::make_pair(x0, y0));
        visited[y0][x0] = true; dist[y0][x0] = 0;

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            xy = queue.front(); queue.pop();
            x0 = xy.first; y0 = xy.second;

            /* If the target is found */
            if(dist[y0][x0] > 1)
            {
                *xs = x0; *ys = y0; return;
            }
            
            /* Expand the point */
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = x0 + dx;
                if(x < 0 || x >= W) continue;
                for(int dy = -1; dy <= 1; dy++)
                {
                    int y = y0 + dy;
                    if(y < 0 || y >= H) continue;
                    if(!visited[y][x])
                    {
                        visited[y][x] = true;
                        queue.push(std::make_pair(x, y));
                        dist[y][x] = map->exp[x][y][0]? 0: dist[y0][x0] + 1;
                    }   
                }   
            }   
        }
    }

    std::vector<std::pair<double, double>> Planner::astar(int xs, int ys, 
                                                          int xg, int yg)
    {
        /* Constants */
        const int X = map->size[simulator::X];
        const int Y = map->size[simulator::Y];
        const double INF = std::numeric_limits<double>::infinity();

        /* Boundaries */
        xs = std::max(std::min(xs, X - 1), 0);
        ys = std::max(std::min(ys, Y - 1), 0);
        xg = std::max(std::min(xg, X - 1), 0);
        yg = std::max(std::min(yg, Y - 1), 0);

        /* Result */
        std::vector<std::pair<double, double>> path;

        /* Obstacle check */
        if(map->exp[xg][yg][0])
            bfs(&xg, &yg);
        if(map->exp[xs][ys][0])
            bfs(&xs, &ys);

        /* Initialize arrays and queue */
        std::vector<double> g(X * Y, INF);
        std::vector<int> parent(X * Y, -1);
        std::vector<bool> visited(X * Y, false);
        std::priority_queue<std::pair<double, int>> queue;

        /* Variables */
        int x, y, x0, y0, idx, index = encode(xs, ys, X);

        /* Push the first point into the queue */
        visited[index] = !(g[index] = 0);
        queue.push(std::make_pair(-f(0, xs, ys, xg, yg), index));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            index = queue.top().second;
            decode(index, &x0, &y0, X);
            visited[index] = true; queue.pop();

            /* If found a path */
            if(x0 == xg && y0 == yg)
            {
                path.push_back(std::make_pair(
                    x0 * map->resolution, 
                    y0 * map->resolution
                ));
                while((index = parent[index]) != -1)
                {
                    decode(index, &x0, &y0, X);
                    path.push_back(std::make_pair(
                        x0 * map->resolution, 
                        y0 * map->resolution
                    ));
                }
                std::reverse(path.begin(), path.end());
                break;
            }

            /* Expand the point */
            for(int neighbor = 0; neighbor < 9; neighbor++)
            {
                x = x0 + neighbor % 3 - 1;
                y = y0 + neighbor / 3 - 1;
                idx = encode(x, y, X);
                
                /* Determine the legitimacy of the extension point */
                if(x < 0 || x >= X || y < 0 || y >= Y || 
                   map->exp[x][y][0] || visited[idx])
                    continue;
                
                /* Calculate cost value */
                double cost = f(g[index], x0, y0, x, y);

                /* Update the point */
                if(cost < g[idx])
                {
                    g[idx] = cost;
                    parent[idx] = index;
                    queue.push(std::make_pair(-f(cost, x, y, xg, yg), idx));
                }
            }
        }

        return path;
    }

    std::vector<std::pair<double, double>> Planner::bspline(
        const std::vector<std::pair<double, double>>& path
    ){
        const int N = path.size();
        Eigen::Vector3d p(3), v(3), a(3);
        Eigen::VectorXd x(N + 4), y(N + 4);
        p << 1, 4, 1; v << -1, 0, 1; a << 1, -2, 1;
        std::vector<std::pair<double, double>> control(N + 2);
        Eigen::MatrixXd m = Eigen::MatrixXd::Zero(N + 4, N + 2);
        for(int i = 0; i < N; i++)
        {
            x[i] = path[i].first; y[i] = path[i].second;
            m.block(i, i, 1, 3) = (1.0 / 6) * p.transpose();
        }
        x.tail(4).setZero(); y.tail(4).setZero();
        m.block(N, 0, 1, 3) = (0.5 / time) * v.transpose();
        m.block(N + 1, N - 1, 1, 3) = (0.5 / time) * v.transpose();
        m.block(N + 2, 0, 1, 3) = (1. / (time * time)) * a.transpose();
        m.block(N + 3, N - 1, 1, 3) = (1. / (time * time)) * a.transpose();
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> c = m.colPivHouseholderQr();
        Eigen::VectorXd cx = c.solve(x), cy = c.solve(y);
        for(int i = 0; i < N + 2; i++)
            control[i] = std::make_pair(cx[i], cy[i]);
        return control;
    }

    double Planner::optimize(std::vector<std::pair<double, double>>& ctrl)
    {
        /* Initialize variables */
        double cost;
        const int n = ctrl.size();
        lbfgs::lbfgs_parameter_t param;
        Eigen::VectorXd var(n << 1);
        for(int i = 0; i < n; i++)
        {
            var[i * 2] = ctrl[i].first;
            var[i * 2 + 1] = ctrl[i].second;
        }
        param.g_epsilon = 0.0;

        /* Optimize */
        lbfgs::lbfgs_optimize(var, cost, [](
            void* self,
            const Eigen::VectorXd& var,  
            Eigen::VectorXd& gradient
        )->double{
            /* Initialize */
            gradient.setZero();
            double g, cost = 0;
            const double lambda = 10;
            const int n = var.size() >> 1;
            Planner* planner = reinterpret_cast<Planner*>(self);
            double t2 = 1. / (planner->time * planner->time);
            double t4 = t2 * t2;

            /* Calculate velocity */
            int m = (n - 1) << 1;
            std::vector<double> vel(m);
            for(int t = 0; t < m; t++)
                vel[t] = var[t + 2] - var[t];
            
            /* Calculate acceleration */
            m -= 2;
            std::vector<double> acc(m);
            for(int t = 0; t < m; t++)
                acc[t] = vel[t + 2] - vel[t];
            
            /* Calculate smoothness cost */
            for(int t = 0; t < m; t++)
            {
                g = 2 * acc[t];
                cost += acc[t] * acc[t];
                gradient[t + 2] -= 2 * g;
                gradient[t + 4] += g;
                gradient[t] += g;
            }

            /* Calculate acceleration cost */
            for(int t = 0; t < m; t += 2)
            {
                g = (std::pow(acc[t], 2) + std::pow(acc[t + 1], 2)) * t4;
                if((g -= planner->am * planner->am) > 0)
                {
                    cost += g;
                    for(int dim = 0; dim < 2; dim++)
                    {
                        g = acc[t + dim] * t4;
                        gradient[t + dim] += g;
                        gradient[t + dim + 4] += g;
                        gradient[t + dim + 2] -= 2 * g;
                    }
                }
            }

            /* Calculate velocity cost */
            m += 2;
            for(int t = 0; t < m; t += 2)
            {
                g = (std::pow(vel[t], 2) + std::pow(vel[t + 1], 2)) * t2;
                if((g -= planner->vm * planner->vm) > 0)
                {
                    cost += g;
                    for(int dim = 0; dim < 2; dim++)
                    {
                        g = vel[t + dim] * t2;
                        gradient[t + dim] -= g;
                        gradient[t + dim + 2] += g;
                    }
                }
            }

            /* Calculate safety cost */
            m -= 4;
            double** sdf = planner->map->sdf;
            for(int t = 6; t < m; t += 2)
            {
                double x = var[t] / planner->map->resolution;
                double y = var[t + 1] / planner->map->resolution;
                x = std::max(std::min(x, planner->map->size[X] - 2.), 0.);
                y = std::max(std::min(y, planner->map->size[Y] - 2.), 0.);

                /* Linear interpolation */
                int x1 = x, y1 = y;
                int x2 = x1 + 1, y2 = y1 + 1;
                double u = x - x1, v = y - y1;
                double u_ = 1. - u, v_ = 1. - v;
                g = u * v * sdf[x2][y2]
                  + u * v_ * sdf[x2][y1]
                  + u_ * v * sdf[x1][y2]
                  + u_ * v_ * sdf[x1][y1];

                /* Calculate gradient */
                if((g = planner->ds - g) > 0)
                {
                    cost += lambda * g * g;
                    gradient[t] += 2 * lambda * g * (
                        + v * sdf[x1][y2] + v_ * sdf[x1][y1]
                        - v * sdf[x2][y2] - v_ * sdf[x2][y1]
                    );
                    gradient[t + 1] += 2 * lambda * g * (
                        + u * sdf[x2][y1] + u_ * sdf[x1][y1]
                        - u * sdf[x2][y2] - u_ * sdf[x1][y2]
                    );
                }
            }

            gradient.head(6).setZero();
            gradient.tail(6).setZero();
            return cost;

        }, nullptr, nullptr, this, param);

        /* Get the optimal control points */
        for(int i = 0; i < n; i++)
            ctrl[i] = std::make_pair(var[i * 2], var[i * 2 + 1]);
        return cost;
    }
}
