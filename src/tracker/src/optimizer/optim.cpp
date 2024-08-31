/* @Author YueLin */

#include "optimizer/optim.hpp"

namespace eva_tracker
{
    const int VEL = 0, ACC = 1;
    const int YAW = 0, XYZ = 1, X = 1, Y = 2, Z = 3, DIM = 4;
    const int SAFE = 0, VIS = 1, OCC = 2, ANGLE = 3, SMOOTH = 4, FEASIBLE = 5;

    Optimizer::Optimizer(ESDF* rc, ESDF* camera, int n, double dt,
                         double vm, double wm, double avm, double awm,
                         double lmd_smoothness, double lmd_feasibility, 
                         double lmd_a, double lmd_d, double lmd_v, double lmd_o)
    {
        time = dt;
        params.g_epsilon = 0.0;
        params.max_iterations = n;

        /* ESDF */
        fov = camera; robot = rc;

        /* Motion limits */
        limit2[VEL][YAW] = wm * wm;
        limit2[VEL][XYZ] = vm * vm;
        limit2[ACC][YAW] = awm * awm;
        limit2[ACC][XYZ] = avm * avm;

        /* Hyperparameters */
        lambda[VIS] = lmd_v;
        lambda[OCC] = lmd_o;
        lambda[SAFE] = lmd_d;
        lambda[ANGLE] = lmd_a;
        lambda[SMOOTH] = lmd_smoothness;
        lambda[FEASIBLE] = lmd_feasibility;
    }

    BSpline Optimizer::optimize(BSpline& curve, Path* target,
                                PointCloud* pc, PointCloud* dp)
    {
        /* Initialize */
        double cost;
        cloud = *pc;
        depth = *dp;
        predict = *target;
        Eigen::VectorXd var = curve.controls();

        /* Optimization */
        iterations = 0;
        double duration = TIME();
        lbfgs::lbfgs_optimize(var, cost, f, nullptr, nullptr, this, params);
        duration = TIME() - duration;
        ROS_INFO("[Opt] Duration: %fs.", duration);
        ROS_INFO(
            "Optimized over. Iterations: %d\n\
            Angle cost: %f\n\
            Safety cost: %f\n\
            Occlusion cost: %f\n\
            Visibility cost: %f\n\
            Smoothness cost: %f\n\
            Feasibility cost: %f\n",
            iterations, costs[ANGLE], costs[SAFE], costs[OCC],
            costs[VIS], costs[SMOOTH], costs[FEASIBLE]
        );

        /* Update B-spline */
        Path ctrl(var.size() / DIM);
        const int n = ctrl.size();
        for(int t = 0; t < n; t++)
        {
            int index = t * DIM;
            ctrl[t].x = var[index + X];
            ctrl[t].y = var[index + Y];
            ctrl[t].z = var[index + Z];
            ctrl[t].yaw = var[index + YAW];
        }
        spline.update(ctrl, time);
        return spline;
    }

    double Optimizer::f(void* optimizer,
                        const Eigen::VectorXd& var,
                        Eigen::VectorXd& gradient)
    {
        /* Initialize */
        int index;
        double sin, cos;
        gradient.setZero();
        const int n = var.size() / DIM;
        double cost[6] = {0, 0, 0, 0, 0, 0};
        Optimizer* self = reinterpret_cast<Optimizer*>(optimizer);

        /* Calculate velocity */
        std::vector<double> vel(DIM * (n - 1));
        for(int t = 0; t < n - 1; t++)
        {
            index = t * DIM;
            vel[index] = clip(var[index + DIM] - var[index]);
            for(int dim = 1; dim < DIM; dim++)
                vel[index + dim] = var[index + dim + DIM] - var[index + dim];
        }

        /* Calculate acceleration */
        std::vector<double> acc(DIM * (n - 2));
        for(int t = 0; t < n - 2; t++)
        {
            index = t * DIM;
            for(int dim = 0; dim < DIM; dim++)
                acc[index + dim] = vel[index + dim + DIM] - vel[index + dim];
        }

        /* Calculate jerk */
        std::vector<double> jerk(DIM * (n - 3));
        for(int t = 0; t < n - 3; t++)
        {
            index = t * DIM;
            for(int dim = 0; dim < DIM; dim++)
                jerk[index + dim] = acc[index + dim + DIM] - acc[index + dim];
        }

        /* Occlusion cost and visibility cost */
        for(int t = 1; t < n - 2; t++)
        {
            index = t * DIM;
            const double x = (
                4 * var[index + X + DIM] +
                var[index + X] + var[index + X + DIM * 2]
            ) / 6, y = (
                4 * var[index + Y + DIM] +
                var[index + Y] + var[index + Y + DIM * 2]
            ) / 6, z = (
                4 * var[index + Z + DIM] +
                var[index + Z] + var[index + Z + DIM * 2]
            ) / 6, yaw = (
                4 * var[index + DIM] + var[index] + var[index + DIM * 2]
            ) / 6;
            sin = std::sin(yaw), cos = std::cos(yaw);

            /* We should ensure that the control points do not collide, 
                not the trajectory points */
            // /* Safety cost */
            // for(pcl::PointXYZ obstacle: self->cloud)
            // {
            //     /* Coordinate of the obstacle */
            //     const double
            //     x0 = obstacle.x - x,
            //     y0 = obstacle.y - y,
            //     z0 = obstacle.z - z;
            //     Point<double> point(
            //         x0 * cos + y0 * sin,
            //         y0 * cos - x0 * sin, z0
            //     );

            //     /* Calculate the safety cost */
            //     double dis = self->robot->get(point); if(dis < O) continue;
            //     dis = std::sqrt(dis); cost[SAFE] += self->lambda[SAFE] * dis;

            //     /* Calculate the gradient of the safety cost */
            //     Point<double> g = self->robot->gradient(point);
            //     Point<double> grad = Point<double>(
            //         g.x * cos - g.y * sin,
            //         g.x * sin + g.y * cos, g.z,
            //         g.x * (y0 * cos - x0 * sin) - g.y * (y0 * sin + x0 * cos)
            //     ) * (self->lambda[SAFE] / (12 * dis));
            //     grad.yaw *= self->lambda[SAFE] / (12 * dis);
            //     gradient[index + X] -= grad.x;
            //     gradient[index + Y] -= grad.y;
            //     gradient[index + Z] -= grad.z;
            //     gradient[index + YAW] += grad.yaw;
            //     gradient[index + X + DIM] -= 4 * grad.x;
            //     gradient[index + Y + DIM] -= 4 * grad.y;
            //     gradient[index + Z + DIM] -= 4 * grad.z;
            //     gradient[index + X + DIM * 2] -= grad.x;
            //     gradient[index + Y + DIM * 2] -= grad.y;
            //     gradient[index + Z + DIM * 2] -= grad.z;
            //     gradient[index + YAW + DIM] += 4 * grad.yaw;
            //     gradient[index + YAW + DIM * 2] += grad.yaw;
            // }

            /* Occlusion cost */
            for(pcl::PointXYZ obstacle: self->depth)
            {
                /* Coordinate of the obstacle */
                const double
                x0 = obstacle.x - x,
                y0 = obstacle.y - y,
                z0 = obstacle.z - z;
                Point<double> point(
                    x0 * cos + y0 * sin,
                    y0 * cos - x0 * sin, z0
                );

                /* Calculate the occlusion cost */
                double occ = self->fov->get(point); if(occ < O) continue;
                cost[OCC] += self->lambda[OCC] * occ;

                /* Calculate the gradient of the occlusion cost */
                Point<double> g = self->fov->gradient(point);
                Point<double> grad = Point<double>(
                    g.x * cos - g.y * sin,
                    g.x * sin + g.y * cos, g.z,
                    g.x * (y0 * cos - x0 * sin) - g.y * (y0 * sin + x0 * cos)
                ) * (self->lambda[OCC] / 6);
                grad.yaw *= self->lambda[OCC] / 6;
                gradient[index + X] -= grad.x;
                gradient[index + Y] -= grad.y;
                gradient[index + Z] -= grad.z;
                gradient[index + YAW] += grad.yaw;
                gradient[index + X + DIM] -= 4 * grad.x;
                gradient[index + Y + DIM] -= 4 * grad.y;
                gradient[index + Z + DIM] -= 4 * grad.z;
                gradient[index + X + DIM * 2] -= grad.x;
                gradient[index + Y + DIM * 2] -= grad.y;
                gradient[index + Z + DIM * 2] -= grad.z;
                gradient[index + YAW + DIM] += 4 * grad.yaw;
                gradient[index + YAW + DIM * 2] += grad.yaw;
            }

            /* Observation cost and angle cost */
            const double x0 = self->predict[t].x - x;
            const double y0 = self->predict[t].y - y;
            const double z0 = self->predict[t].z - z;
            Point<double> point(x0 * cos + y0 * sin, y0 * cos - x0 * sin, z0);

            /* Calculate the observation cost */
            cost[VIS] += self->lambda[VIS] * (
                self->fov->max() - self->fov->get(point)
            );

            /* Calculate the gradient of the observation cost */
            Point<double> g = self->fov->gradient(point);
            Point<double> grad = Point<double>(
                g.x * cos - g.y * sin,
                g.y * cos + g.x * sin, g.z,
                g.x * (y0 * cos - x0 * sin) - g.y * (y0 * sin + x0 * cos)
            ) * (self->lambda[VIS] / 6);
            grad.yaw *= self->lambda[VIS] / 6;
            gradient[index + X] += grad.x;
            gradient[index + Y] += grad.y;
            gradient[index + Z] += grad.z;
            gradient[index + YAW] -= grad.yaw;
            gradient[index + X + DIM] += 4 * grad.x;
            gradient[index + Y + DIM] += 4 * grad.y;
            gradient[index + Z + DIM] += 4 * grad.z;
            gradient[index + X + DIM * 2] += grad.x;
            gradient[index + Y + DIM * 2] += grad.y;
            gradient[index + Z + DIM * 2] += grad.z;
            gradient[index + YAW + DIM] -= 4 * grad.yaw;
            gradient[index + YAW + DIM * 2] -= grad.yaw;

            /* Calculate the angle cost */
            double best = std::atan2(y0, x0);
            cost[ANGLE] += self->lambda[ANGLE] * (
                1 - std::cos(var[index] - best)
            );

            /* Calculate the gradient of the angle cost */
            grad.yaw = self->lambda[ANGLE] * std::sin(var[index] - best);
            grad.x = grad.yaw / (x0 * x0 + y0 * y0);
            grad.y = grad.x * x0; grad.x *= y0;
            gradient[index + X] -= grad.x;
            gradient[index + Y] -= grad.y;
            gradient[index + YAW] += grad.yaw;
            gradient[index + X + DIM] -= 4 * grad.x;
            gradient[index + Y + DIM] -= 4 * grad.y;
            gradient[index + X + DIM * 2] -= grad.x;
            gradient[index + Y + DIM * 2] -= grad.y;
            gradient[index + YAW + DIM] += 4 * grad.yaw;
            gradient[index + YAW + DIM * 2] += grad.yaw;
        }

        /* Safety cost */
        for(int t = 0; t < n; t++)
        {
            index = t * DIM;
            const double yaw = var[index],
            x = var[index + X], y = var[index + Y], z = var[index + Z];
            sin = std::sin(yaw), cos = std::cos(yaw);
            for(pcl::PointXYZ obstacle: self->cloud)
            {
                /* Coordinate of the obstacle */
                const double
                x0 = obstacle.x - x,
                y0 = obstacle.y - y,
                z0 = obstacle.z - z;
                Point<double> point(
                    x0 * cos + y0 * sin,
                    y0 * cos - x0 * sin, z0
                );

                /* Calculate the safety cost */
                double dis = self->robot->get(point); if(dis < O) continue;
                dis = std::sqrt(dis); cost[SAFE] += self->lambda[SAFE] * dis;

                /* Calculate the gradient of the safety cost */
                dis *= 2;
                Point<double> g = self->robot->gradient(point);
                Point<double> grad = Point<double>(
                    g.x * cos - g.y * sin,
                    g.x * sin + g.y * cos, g.z,
                    g.x * (y0 * cos - x0 * sin) - g.y * (y0 * sin + x0 * cos)
                ) * (self->lambda[SAFE] / dis);
                gradient[index + X] -= grad.x;
                gradient[index + Y] -= grad.y;
                gradient[index + Z] -= grad.z;
                gradient[index + YAW] += grad.yaw * self->lambda[SAFE] / dis;
            }
        }

        /* Velocity feasibility cost */
        double t2 = 1 / pow2(self->time);
        for(int t = 0; t < n - 1; t++)
        {
            index = t * DIM;
            for(int axis = 0; axis < DIM; axis++)
            {
                double v = pow2(vel[index + axis]) * t2;
                double m = self->limit2[VEL][axis? XYZ: YAW];
                if(v <= m) continue;
                double g = 2 * self->lambda[FEASIBLE] * vel[index + axis] * t2;
                cost[FEASIBLE] += self->lambda[FEASIBLE] * (v - m);
                gradient[index + axis + DIM] += g;
                gradient[index + axis] -= g;
            }
        }

        /* Acceleration feasibility cost */
        double t4 = t2 * t2;
        for(int t = 0; t < n - 2; t++)
        {
            index = t * DIM;
            for(int axis = 0; axis < DIM; axis++)
            {
                double a = pow2(acc[index + axis]) * t4;
                double m = self->limit2[ACC][axis? XYZ: YAW];
                if(a <= m) continue;
                double g = 2 * self->lambda[FEASIBLE] * acc[index + axis] * t4;
                cost[FEASIBLE] += self->lambda[FEASIBLE] * (a - m);
                gradient[index + axis + DIM] -= 2 * g;
                gradient[index + axis + DIM * 2] += g;
                gradient[index] += g;
            }
        }

        /* Smoothness cost */
        double t6 = t4 * t2;
        for(int t = 0; t < n - 3; t++)
        {
            index = t * DIM;
            for(int axis = 0; axis < DIM; axis++)
            {
                cost[SMOOTH] += self->lambda[SMOOTH] * t6 * pow2(
                    jerk[index + axis]
                );
                double g = 2 * self->lambda[SMOOTH] * jerk[index + axis] * t6;
                gradient[index + axis + DIM * 2] -= 3 * g;
                gradient[index + axis + DIM * 3] += g;
                gradient[index + axis + DIM] += 3 * g;
                gradient[index + axis] -= g;
            }
        }

        /* Do not optimize starting point */
        const int order = DIM * self->spline.deg();
        for(int d = 0; d < order; d++)
            gradient[d] = 0;

        /* Sum all costs */
        double sum = 0;
        self->iterations++;
        for(int c = 0; c < 6; c++)
        {
            sum += cost[c];
            self->costs[c] = cost[c];
        }
        return sum;
    }
}
