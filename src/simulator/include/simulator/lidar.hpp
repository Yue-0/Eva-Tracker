/* @Author: YueLin */

#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "sensor_msgs/PointCloud2.h"

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace simulator
{
    /* Convert Map to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 map2msg(Map&, std::string&);

    /* Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 cloud2msg(
        pcl::PointCloud<pcl::PointXYZ>&, std::string&
    );

    /* Create a multi-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle&, std::string&,
                     Map&, Robot&, ros::Publisher&,
                     double, double&, double&, double&, int&);
    
    /* Create a single-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle&, std::string&, Map&,
                     Robot&, ros::Publisher&, double, double&, int&);
    
    /* Create a depth camera */
    ros::Timer depth(ros::NodeHandle&, std::string&,
                     Map&, Robot&, ros::Publisher&,
                     double, double&, double&, double&, double&, int&);
    
    /* Visualize filed of view */
    nav_msgs::Path FoV(std::string, double, double, double);
}
