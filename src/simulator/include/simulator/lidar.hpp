/* @Author: YueLin */

#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "sensor_msgs/PointCloud2.h"

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace sim
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud3D;

    /* Convert Map to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 map2msg(Map&, std::string&);

    /* Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 cloud2msg(PointCloud3D&, std::string&);

    /* Create a multi-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle&, std::string&,
                     Map&, Robot&, ros::Publisher&,
                     double, double&, double&, double&, int&);
    
    /* Create a single-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle&, std::string&, Map&,
                     Robot&, ros::Publisher&, double, double&, int&);
}
