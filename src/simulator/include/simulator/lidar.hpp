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
    sensor_msgs::PointCloud2 map2msg(Map& map, std::string& frame);

    /* Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 cloud2msg(
        pcl::PointCloud<pcl::PointXYZ>& cloud, std::string& frame
    );

    /* Create a multi-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle& n, 
                     std::string& frame,
                     Map& map, Robot& robot, 
                     ros::Publisher& publisher,
                     double time, double& total, 
                     double& height, double& angle, int& range);
    
    /* Create a single-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle& n, 
                     std::string& frame, 
                     Map& map, Robot& robot, 
                     ros::Publisher& publisher, 
                     double time, double& angle, int& range);
    
    /* Create a depth camera */
    ros::Timer depth(ros::NodeHandle& n, 
                     std::string& frame,
                     Map& map, Robot& robot, 
                     ros::Publisher& publisher,
                     double time, double& total, double& height, 
                     double& angles, double& angle, int& range);
    
    /* Visualize filed of view */
    nav_msgs::Path FoV(std::string frame, 
                       double distance, 
                       double alpha, 
                       double beta);
}
