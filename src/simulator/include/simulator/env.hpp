/* @Author: YueLin */

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace simulator
{
    class Environment
    {
        public:
            Map* map;
            Robot target;
            Robot tracker;
        
        public:
            Environment(Map* ptr,
                        double target_x,
                        double target_y,
                        double target_z,
                        double target_yaw,
                        double target_w,
                        double target_l,
                        double target_h,
                        double tracker_x,
                        double tracker_y,
                        double tracker_z,
                        double tracker_yaw,
                        double tracker_w,
                        double tracker_l,
                        double tracker_h): map(ptr)
            {
                target = Robot({
                    target_x, target_y, target_z, target_yaw
                }, target_l, target_w, target_h);
                tracker = Robot({
                    tracker_x, tracker_y, tracker_z, tracker_yaw
                }, tracker_l, tracker_w, tracker_h);
            }
        
        private:
            void crop(Robot& robot)
            {
                robot.pose.x() = std::max(
                    std::min(robot.pose.x(), map->size0.x()), 0.
                );
                robot.pose.y() = std::max(
                    std::min(robot.pose.y(), map->size0.y()), 0.
                );
                robot.pose.z() = std::max(
                    std::min(robot.pose.z(), map->size0.z()), 0.
                );
            }

        public:
            void step(double dt)
            {
                target.move(dt); crop(target);
                tracker.move(dt); crop(tracker);
            }
        
        /* For benchmarking */
        public:
            double angle()
            {
                return std::fabs(clip(std::atan2(
                    target.pose.y() - tracker.pose.y(),
                    target.pose.x() - tracker.pose.x()
                ) - tracker.pose.w()));
            }

            Eigen::Vector2d project()
            {
                double sin = std::sin(tracker.pose.w());
                double cos = std::cos(tracker.pose.w());
                double x = target.pose.x() - tracker.pose.x();
                double y = target.pose.y() - tracker.pose.y();
                return {cos * x + sin * y, cos * y - sin * x};
            }
            
            bool occlusion()
            {
                int x, y, z;
                double r = 1. / map->resolution;
                for(double p, k = 0; k <= 1; k += map->resolution)
                {
                    p = 1 - k;
                    x = std::round(
                        r * (k * tracker.pose.x() + p * target.pose.x())
                    );
                    y = std::round(
                        r * (k * tracker.pose.y() + p * target.pose.y())
                    );
                    z = std::round(
                        r * (k * tracker.pose.z() + p * target.pose.z())
                    );
                    if(map->map[x][y][z]) return true;
                }
                return false;
            }

            double distance()
            {
                return (target.pose.head(3) - tracker.pose.head(3)).norm();
            }
    };
}
