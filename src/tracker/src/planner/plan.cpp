/* @Author YueLin */

#include "planner/plan.hpp"

namespace eva_tracker
{
    bool PathPlanner::visible(Eigen::Vector3d tracker, 
                              const Eigen::Vector3d& target) const
    {
        tracker -= target;
        for(double t = map->resolution; t <= 1; t += map->resolution)
            if(map->get(target + t * tracker))
                return false;
        return true;
    }

    Eigen::Matrix4Xd PathPlanner::plan(Eigen::Vector4d start,
                                       const Eigen::Matrix3Xd& prediction) const
    {
        /* If no target prediction */
        const int n = prediction.cols();
        if(n <= 0) return start;
        
        /* Initialize path */
        Eigen::Matrix4Xd path(4, n);
        path.col(0) = start;

        /* Visibility-aware path generation algorithm */
        for(int p = 1; p < n; path.col(p++) = start)
        {
            /* Generate candidate observation point */
            const Eigen::Vector3d& target = prediction.col(p);
            Eigen::Vector3d observation = target;
            observation.head(2) -= distance * (
                target.head(2) - start.head(2)
            ).normalized();

            /* Calculate yaw angle */
            double angle = 0;
            int direction = 0;
            start.w() = std::atan2(
                observation.y() - target.y(), 
                observation.x() - target.x()
            );

            /* Check visibility */
            if(!feasible(target, start.head(3), observation))
                while((angle += step) <= RA)
                {
                    observation.head(2) = distance * Eigen::Vector2d(
                        std::cos(start.w() + angle), std::sin(start.w() + angle)
                    ) + target.head(2);
                    if(feasible(target, start.head(3), observation))
                    {
                        direction = 1; break;
                    }
                    
                    observation.head(2) = distance * Eigen::Vector2d(
                        std::cos(start.w() - angle), std::sin(start.w() - angle)
                    ) + target.head(2);
                    if(feasible(target, start.head(3), observation))
                    {
                        direction = -1; break;
                    }
                }

            /* If no visible observation point */
            if(angle >= RA && !direction)
                return path.leftCols(p);

            /* Update pose of the tracker */
            start.head(3) = observation;
            start.w() += direction * angle + PI;
            if(start.w() < 0 || start.w() >= 2 * PI)
                start.w() -= 2 * PI * std::floor(start.w() / (2 * PI));
        }
        return path;
    }
}
