/* @Author YueLin */

#include "planner/map.hpp"

namespace eva_tracker
{
    class PathPlanner
    {
        private:
            Map* map;
            double distance, step;
            const double PI = std::acos(-1);
            const double RA = PI / 2;

        public:
            PathPlanner(Map* world, double d, double a): 
                map(world), distance(d), step(a) {}
        
        public:
            bool visible(Eigen::Vector3d tracker, 
                         const Eigen::Vector3d& target) const;
            
            Eigen::Matrix4Xd plan(Eigen::Vector4d start, 
                                  const Eigen::Matrix3Xd& prediction) const;

        private:
            bool feasible(const Eigen::Vector3d& target, 
                          const Eigen::Vector3d& tracker,
                          const Eigen::Vector3d& observation) const
            {
                return visible(observation, target)
                    && visible(observation, tracker);
            }
    };
}
