/* @Author: YueLin */

#include "simulator/env.hpp"

namespace sim
{
    bool Environment::step(double dt)
    {
        return step(dt, target), step(dt, tracker);
    }

    void Environment::clip(Robot& robot)
    {
        robot.pose.x = std::max(std::min(robot.pose.x, map.size0[X]), 0.);
        robot.pose.y = std::max(std::min(robot.pose.y, map.size0[Y]), 0.);
        robot.pose.z = std::max(std::min(robot.pose.z, map.size0[Z]), 0.);
    }

    bool Environment::step(double dt, Robot& robot)
    {
        double r = 1 / map.resolution,
        x = robot.pose.x, y = robot.pose.y,
        yaw = robot.pose.yaw; robot.move(dt); int
        x0 = std::max(std::min(round(robot.pose.x * r), map.size[X] - 1.), 0.),
        y0 = std::max(std::min(round(robot.pose.y * r), map.size[Y] - 1.), 0.),
        z0 = std::max(std::min(round(robot.pose.z * r), map.size[Z] - 1.), 0.);
        double sin = std::sin(robot.pose.yaw), cos = std::cos(robot.pose.yaw),
        h = (robot.length * sin + robot.width * cos) / (map.resolution * 2),
        w = (robot.length * cos + robot.width * sin) / (map.resolution * 2);
        int x1 = std::max(x0 - w, 0.), x2 = std::min(x0 + w, map.size[X] - 1.);
        int y1 = std::max(y0 - h, 0.), y2 = std::min(y0 + h, map.size[Y] - 1.);
        double tan, cot, b1, b2; bool rotate;
        if((rotate = std::min(std::fabs(cos), std::fabs(sin)) > 1e-6))
        {
            tan = sin / cos, cot = cos / sin;
            b1 = tan * x0 - y0, b2 = cot * x0 + y0;
            w = r * robot.width * std::sqrt(cot * cot + 1);
            h = r * robot.length * std::sqrt(tan * tan + 1);
        }
        for(int i = 1 + x1; i < x2; i++)
            for(int j = 1 + y1; j < y2; j++)
            {
                if(rotate && ((j - i * tan + b1) > h || (j + i * cot - b2) > w))
                    continue;
                if(map.map[i][j][z0])
                {
                    robot.pose.x = x;
                    robot.pose.y = y;
                    robot.pose.yaw = yaw;
                    return false;
                }
            }
        return clip(robot), true;
    }

    Environment::Environment(double resolution,
                             double map_x,
                             double map_y,
                             double map_z,
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
                             double tracker_h)
    {
        map = Map(map_x, map_y, map_z, resolution);
        SO3 target_pose = {target_x, target_y, target_z, target_yaw};
        SO3 tracker_pose = {tracker_x, tracker_y, tracker_z, tracker_yaw};
        tracker = Robot(tracker_pose, tracker_l, tracker_w, tracker_h);
        target = Robot(target_pose, target_l, target_w, target_h);
    }
}
