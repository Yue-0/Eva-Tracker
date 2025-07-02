/* @Author YueLin */

#include "planner/path.hpp"

namespace eva_tracker
{
    bool PathGenerator::visible(Map& map, 
                                Eigen::Vector3d target, 
                                Eigen::Vector3d tracker)
    {
        Eigen::Vector3d delta = tracker - target;
        for(double t = map.resolution; t <= 1; t += map.resolution)
            if(map.get(target + t * delta))
                return false;
        return true;
    }

    Eigen::Matrix4Xd PathGenerator::generate(Map& map, 
                                             Eigen::Vector4d start,
                                             Eigen::Matrix3Xd prediction)
    {
        /* Initialize */
        const int n = prediction.cols();
        Eigen::Matrix4Xd path(4, std::max(n, 0));

        /* If no target */
        path.col(0) = start;
        if(n <= 0) return path.block(0, 0, 4, 1);

        /* Visibility-aware path generation algorithm */
        for(int p = 1; p < n; p++)
        {
            /* Generate candidate observation point */
            Eigen::Vector3d observation, target = prediction.col(p);
            observation.head(2) = target.head(2) - distance * (
                target.head(2) - start.head(2)
            ).normalized();
            observation[Z] = target[Z];

            /* Calculate yaw angle */
            int direction = 0;
            double angle = 0., theta = std::atan2(
                observation[Y] - target[Y], 
                observation[X] - target[X]
            );

            /* Check visibility */
            if(!visible(map, target, observation, start.head(3)))
                while((angle += step) <= RA)
                {
                    observation.head(2) = distance * Eigen::Vector2d(
                        std::cos(theta + angle), std::sin(theta + angle)
                    ) + target.head(2);
                    if(visible(map, target, observation, start.head(3)))
                    {
                        direction = 1; break;
                    }
                    
                    observation.head(2) = distance * Eigen::Vector2d(
                        std::cos(theta - angle), std::sin(theta - angle)
                    ) + target.head(2);
                    if(visible(map, target, observation, start.head(3)))
                    {
                        direction = -1; break;
                    }
                }

            /* If no visible observation point */
            if(angle >= RA && !direction)
                return path.block(0, 0, 4, p - 1);

            /* Calculate observation yaw angle */
            theta += direction * angle + PI;
            if(theta < 0 || theta >= 2 * PI)
                theta -= 2 * PI * std::floor(theta / (2 * PI));

            /* Update path */
            path.block(0, p, 3, 1) = observation;
            path(3, p) = theta; start = path.col(p);
        }
        return path;
    }
}
