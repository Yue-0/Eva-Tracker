/* @Author: YueLin */

#include <limits>

#include "sensor_msgs/LaserScan.h"

#include "simulator/lidar.hpp"

const double INF = std::numeric_limits<double>::infinity();

namespace sim
{
    sensor_msgs::PointCloud2 map2msg(Map& map, std::string& frame)
    {
        /* Map -> Point cloud */
        PointCloud3D cloud;
        for(int x = 0; x < map.size[X]; x++)
            for(int y = 0; y < map.size[Y]; y++)
                for(int z = 0; z < map.size[Z]; z++)
                    if(map.map[x][y][z])
                        cloud.push_back(pcl::PointXYZ(
                            x * map.resolution,
                            y * map.resolution,
                            z * map.resolution
                        ));
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.width = cloud.points.size();
        
        /* Point cloud -> Ros message */
        return cloud2msg(cloud, frame);
    }

    sensor_msgs::PointCloud2 cloud2msg(PointCloud3D& cloud, std::string& frame)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = frame;
        return msg;
    }

    ros::Timer LiDAR(ros::NodeHandle& n,
                     std::string& frame,
                     Map& map, Robot& robot,
                     ros::Publisher& publisher,
                     double time, double& total, 
                     double& height, double& angle, int& range)
    {
        static double pi = PI;
        return depth(
            n, frame, map, robot, publisher, 
            time, total, height, pi, angle, range
        );
    }

    ros::Timer LiDAR(ros::NodeHandle& n,
                     std::string& frame,
                     Map& map, Robot& robot,
                     ros::Publisher& publisher,
                     double time, double& angle, int& range)
    {
        return n.createTimer(ros::Duration(time), [&](const ros::TimerEvent&){
            /* Position */
            sensor_msgs::LaserScan cloud;
            double yaw = robot.pose.yaw;
            double r = 1 / map.resolution;
            int x0 = std::round(robot.pose.x * r);
            int y0 = std::round(robot.pose.y * r);
            int z0 = std::round(robot.pose.z * r);

            /* Initialize */
            cloud.range_min = 0;
            cloud.angle_max = PI;
            cloud.angle_min = angle - PI;
            cloud.angle_increment = angle;
            cloud.header.frame_id = frame;
            cloud.range_max = 2 * std::max(map.size0[X], map.size0[Y]);
            
            /* Laser scan */
            cloud.ranges.clear();
            cloud.intensities.clear();
            for(float rad = angle - PI; rad <= PI; rad += angle)
            {
                bool scan = false;
                int t = -1; double
                sin = std::sin(rad + yaw),
                cos = std::cos(rad + yaw);
                while(++t <= range)
                {
                    int x = std::round(x0 + t * cos);
                    int y = std::round(y0 + t * sin);
                    if(x < 0 || x >= map.size[X])
                    {
                        cloud.ranges.push_back(INF);
                        scan = true; break;
                    }
                    if(y < 0 || y >= map.size[Y])
                    {
                        cloud.ranges.push_back(INF);
                        scan = true; break;
                    }
                    if(map.map[x][y][z0])
                    {
                        cloud.ranges.push_back(t * map.resolution);
                        scan = true; break;
                    }
                }
                if(!scan) cloud.ranges.push_back(INF);
            }

            /* Publish */
            publisher.publish(cloud);
        });
    }

    ros::Timer depth(ros::NodeHandle& n,
                     std::string& frame,
                     Map& map, Robot& robot,
                     ros::Publisher& publisher,
                     double time, double& total, double& height, 
                     double& angles, double& angle, int& range)
    {
        return n.createTimer(ros::Duration(time), [&](const ros::TimerEvent&){
            /* Position */
            double r = 1 / map.resolution;
            int x0 = std::round(robot.pose.x * r);
            int y0 = std::round(robot.pose.y * r);
            int z0 = std::round(robot.pose.z * r);

            /* Initialize */
            int dz = height * r;
            int h0 = (total * r) / 2;
            int z1 = std::max(z0 - h0, 0);
            int z2 = std::min(z0 + h0, map.size[Z] - 1);
            
            /* Laser scan */
            PointCloud3D cloud;
            for(float rad = -angles; rad < angles; rad += angle)
            {
                double sin = std::sin(rad + robot.pose.yaw);
                double cos = std::cos(rad + robot.pose.yaw);
                double sin0 = std::sin(rad), cos0 = std::cos(rad);
                for(int z = z1; z <= z2; z += dz)
                {
                    int t = 0;
                    while(++t <= range)
                    {
                        int x = std::round(x0 + t * cos);
                        int y = std::round(y0 + t * sin);
                        if(x < 0 || x >= map.size[X]) break;
                        if(y < 0 || y >= map.size[Y]) break;
                        if(map.map[x][y][z])
                        {
                            cloud.push_back(pcl::PointXYZ(
                                t * map.resolution * cos0,
                                t * map.resolution * sin0,
                                (z - z0) * map.resolution
                            ));
                            break;
                        }
                    }
                }
            }
            cloud.height = 1;
            cloud.is_dense = true;
            cloud.width = cloud.points.size();

            /* Publish */
            publisher.publish(cloud2msg(cloud, frame));
        });
    }
}
