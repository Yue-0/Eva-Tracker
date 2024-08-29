/* @Author: YueLin */

#include "predictor/predictor.hpp"

namespace trajectory_prediction
{
    const int X = 0, Y = 1, Z = 2;
    const int DIM = X + Y + Z;

    Predictor::Predictor(double weight, int n)
    {
        gamma = weight;
        exp.push_back(1);
        double e = std::exp(1);
        params.max_iterations = n;
        for(int pow = 0; pow < n; pow++)
            exp.push_back(e * exp.back());
        std::reverse(exp.begin(), exp.end());
    }

    Points Predictor::predict(Points& path)
    {
        /* Expand */
        prediction = path;
        observation = path;
        int len = path.size();
        if(len <= 1) return Points();
        eva_tracker::Point<double> delta = path[len - 1] - path[len - 2];
        for(int t = len - 2; t >= 0; t--)
            prediction.push_back(prediction.back() + delta);
        
        /* Optimize preprocessing */
        int index;
        double cost;
        len = (len << 1) - 1;
        Eigen::VectorXd var(len * DIM);
        for(int t = 0; t < len; t++)
        {
            index = t * DIM;
            var[index + X] = prediction[t].x;
            var[index + Y] = prediction[t].y;
            var[index + Z] = prediction[t].z;
        }

        /* Optimization */
        lbfgs::lbfgs_optimize(var, cost, f, nullptr, nullptr, this, params);

        /* Optimize post-processing */
        for(int t = 0; t < len; t++)
        {
            index = t * DIM;
            prediction[t].x = var[index + X];
            prediction[t].y = var[index + Y];
            prediction[t].z = var[index + Z];
        }

        /* Return prediction result */
        Points result;
        result.assign(prediction.begin() + ((len + 1) >> 1), prediction.end());
        return result;
    }

    double Predictor::f(void* predictor,
                        const Eigen::VectorXd& var,
                        Eigen::VectorXd& gradient)
    {
        double cost1 = 0, cost2 = 0; 
        Predictor* self = reinterpret_cast<Predictor*>(predictor);

        /* Calculate cost and gradient */
        double dx, dy, dz;
        gradient.setZero();
        const int len = self->prediction.size() - 2;
        for(int p = 0; p < len; p++)
        {
            int idx = p * DIM;

            /* Observation cost and gradient */
            if(len >> 1 >= p)
            {
                dx = self->observation[p].x - var[idx + X];
                dy = self->observation[p].y - var[idx + Y];
                dz = self->observation[p].z - var[idx + Z];

                gradient[idx + X] += 2 * self->exp[p] * dx;
                gradient[idx + Y] += 2 * self->exp[p] * dy;
                gradient[idx + Z] += 2 * self->exp[p] * dz;

                cost1 += self->exp[p] * (dx * dx + dy * dy + dz * dz);
            }

            /* Smooth cost */
            dx = var[idx + X + DIM * 2] - 2 * var[idx + X + DIM] + var[idx + X];
            dy = var[idx + Y + DIM * 2] - 2 * var[idx + Y + DIM] + var[idx + Y];
            dz = var[idx + Z + DIM * 2] - 2 * var[idx + Z + DIM] + var[idx + Z];
            cost2 += self->gamma * (dx * dx + dy * dy + dz * dz) / 2;

            /* Smooth gradient */
            dx *= self->gamma;
            dy *= self->gamma;
            dz *= self->gamma;
            gradient[idx + X] += dx;
            gradient[idx + Y] += dy;
            gradient[idx + Z] += dz;
            gradient[idx + X + DIM] -= 2 * dx;
            gradient[idx + Y + DIM] -= 2 * dy;
            gradient[idx + Z + DIM] -= 2 * dz;
            gradient[idx + X + DIM * 2] += dx;
            gradient[idx + Y + DIM * 2] += dy;
            gradient[idx + Z + DIM * 2] += dz;
        }

        return cost1 + cost2;
    }
}
