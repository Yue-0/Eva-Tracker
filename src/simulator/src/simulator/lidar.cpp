/* @Author: YueLin */

#include <limits>

#include "sensor_msgs/LaserScan.h"

#include "simulator/lidar.hpp"

const double INF = std::numeric_limits<double>::infinity();

namespace simulator
{
    sensor_msgs::PointCloud2 map2msg(Map& map, std::string& frame)
    {
        /* Map -> Point cloud */
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for(int x = 0; x < map.size.x(); x++)
            for(int y = 0; y < map.size.y(); y++)
                for(int z = 0; z < map.size.z(); z++)
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

    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ>& cloud, 
                                       std::string& frame)
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
        if(range) return depth(
            n, frame, map, robot, publisher, 
            time, total, height, pi, angle, range
        );
        return n.createTimer(
            ros::Duration(time), [](const ros::TimerEvent&){}, true
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
            double yaw = robot.pose.w();
            double r = 1 / map.resolution;
            int x0 = std::round(robot.pose.x() * r);
            int y0 = std::round(robot.pose.y() * r);
            int z0 = std::round(robot.pose.z() * r);

            /* Initialize */
            cloud.range_min = 0;
            cloud.angle_max = PI;
            cloud.angle_min = angle - PI;
            cloud.angle_increment = angle;
            cloud.header.frame_id = frame;
            cloud.range_max = 2 * std::max(map.size0.x(), map.size0.y());
            
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
                    if(x < 0 || x >= map.size.x())
                    {
                        cloud.ranges.push_back(INF);
                        scan = true; break;
                    }
                    if(y < 0 || y >= map.size.y())
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
            int x0 = std::round(robot.pose.x() * r);
            int y0 = std::round(robot.pose.y() * r);
            int z0 = std::round(robot.pose.z() * r);

            /* Initialize */
            int dz = height * r;
            int h0 = (total * r) / 2;
            int z1 = std::max(z0 - h0, 0);
            int z2 = std::min(z0 + h0, map.size.z() - 1);
            
            /* Laser scan */
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for(float rad = -angles; rad < angles; rad += angle)
            {
                double sin = std::sin(rad + robot.pose.w());
                double cos = std::cos(rad + robot.pose.w());
                for(int z = z1; z <= z2; z += dz)
                {
                    int t = 0;
                    while(++t <= range)
                    {
                        int x = std::round(x0 + t * cos);
                        int y = std::round(y0 + t * sin);
                        if(x < 0 || x >= map.size.x()) break;
                        if(y < 0 || y >= map.size.y()) break;
                        if(map.map[x][y][z])
                        {
                            cloud.push_back(pcl::PointXYZ(
                                t * map.resolution * cos + robot.pose.x(),
                                t * map.resolution * sin + robot.pose.y(),
                                z * map.resolution
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

    nav_msgs::Path FoV(std::string frame,
                       double distance, 
                       double alpha,
                       double beta)
    {
        nav_msgs::Path fov;
        geometry_msgs::PoseStamped poses[5];
        poses[4].header.frame_id = frame;
        fov.header.frame_id = frame;
        const double z = distance * std::tan(beta);
        const double y = distance * std::tan(alpha);
        for(int p = 0; p < 4; p++)
        {
            poses[p].pose.position.x = distance;
            poses[p].pose.position.y = y * (p & 1? 1: -1);
            poses[p].pose.position.z = z * (p >> 1? 1: -1);
            poses[p].header.frame_id = fov.header.frame_id;
            fov.poses.push_back(poses[4]);
            fov.poses.push_back(poses[p]);
        }
        poses[2].pose.position.y *= -1;
        poses[3].pose.position.y *= -1;
        for(int p = 4; p; fov.poses.push_back(poses[--p]));
        fov.poses.push_back(poses[3]);
        return fov;
    }
}
