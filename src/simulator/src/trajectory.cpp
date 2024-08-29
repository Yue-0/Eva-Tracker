/* @Author: YueLin */

#include <cmath>

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#define O 1e-3
#define TARGET 0
#define TRACKER 1

#define NODE "simulation/trajectory"
#define ROS_INIT(node) ros::init(argc, argv, node)

#define callback(robot) [&](nav_msgs::Odometry::ConstPtr odom){\
    geometry_msgs::PoseStamped pose;\
    pose.pose = odom->pose.pose;\
    pose.header = odom->header;\
    if(trajectories[robot].poses.size() > 1){\
        geometry_msgs::Point p =\
        trajectories[robot].poses.back().pose.position;\
        if(std::fabs(odom->pose.pose.position.x - p.x) < O &&\
           std::fabs(odom->pose.pose.position.y - p.y) < O &&\
           std::fabs(odom->pose.pose.position.z - p.z) < O) return;\
    }\
    trajectories[robot].poses.push_back(pose);\
    publisher[robot].publish(trajectories[robot]);\
}

int main(int argc, char* argv[])
{
    /* Initialize */
    ROS_INIT(NODE);
    ros::NodeHandle nh("~");
    nav_msgs::Path trajectories[2];
    nh.getParam("map/frame", trajectories[TARGET].header.frame_id);
    nh.getParam("map/frame", trajectories[TRACKER].header.frame_id);

    /* Publishers */
    ros::Publisher publisher[2];
    publisher[TARGET] = nh.advertise<nav_msgs::Path>("/target/trajectory", 1);
    publisher[TRACKER] = nh.advertise<nav_msgs::Path>("/tracker/trajectory", 1);
    
    /* Subscribers */
    ros::Subscriber target = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom", 1, callback(TARGET)
    );
    ros::Subscriber tracker = nh.subscribe<nav_msgs::Odometry>(
        "/tracker/odom", 1, callback(TRACKER)
    );

    return ros::spin(), 0;
}
