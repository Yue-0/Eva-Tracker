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
    sensor_msgs::PointCloud2 map2msg(const Map& map, const std::string& frame);

    /* Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2 */
    sensor_msgs::PointCloud2 cloud2msg(
        const pcl::PointCloud<pcl::PointXYZ>& c, const std::string& frame
    );

    /* Create a multi-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle& n, 
                     const std::string& frame,
                     const ros::Publisher& publisher,
                     const Map& map, const Robot& robot, 
                     const double time, const double& total, 
                     const double& dh, const double& angle, const int& range);
    
    /* Create a single-line LiDAR */
    ros::Timer LiDAR(ros::NodeHandle& n, 
                     const std::string& frame, 
                     const ros::Publisher& publisher, 
                     const Map& map, const Robot& robot, 
                     const double time, const double& angle, const int& range);
    
    /* Create a depth camera */
    ros::Timer depth(ros::NodeHandle& n, 
                     const std::string& frame,
                     const ros::Publisher& publisher,
                     const Map& map, const Robot& robot, 
                     const double time, const double& total, const double& dh, 
                     const double& angles, const double& da, const int& range);
    
    /* Visualize filed of view */
    nav_msgs::Path FoV(const std::string& frame, 
                       const double distance, 
                       const double alpha, 
                       const double beta);
}
