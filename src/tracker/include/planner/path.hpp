/* @Author YueLin */

#include "planner/map.hpp"

namespace eva_tracker
{
    class PathGenerator
    {
        private:
            double distance, step;
            const int X = 0, Y = 1, Z = 2;
            const double PI = std::acos(-1);
            const double RA = PI / 2;

        public:
            PathGenerator(double d, double a): distance(d), step(a) {};
        
        public:
            bool visible(Map&, Eigen::Vector3d, Eigen::Vector3d);
            Eigen::Matrix4Xd generate(Map&, Eigen::Vector4d, Eigen::Matrix3Xd);

        private:
            bool visible(Map& map, Eigen::Vector3d& target, 
                         Eigen::Vector3d& observation,
                         Eigen::Vector3d tracker)
            {
                return visible(map, target, observation)
                    && visible(map, tracker, observation);
            }
    };
}
