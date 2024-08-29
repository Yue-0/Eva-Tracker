/* @Author: YueLin */

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#define release(lock) lock = false
#define wait4lock(lock) while(lock) ros::Duration(1e-4).sleep(); lock = true

#define ROS_INIT(node) ros::init(argc, argv, node)

int main(int argc, char* argv[])
{
    ROS_INIT("SOT");
    bool lock = false;
    nav_msgs::Path path;
    ros::NodeHandle nh("~");
    geometry_msgs::PoseStamped pose;
    nh.getParam("map/frame", path.header.frame_id);
    nh.getParam("map/frame", pose.header.frame_id);
    unsigned int fps = nh.param("tracker/fps", 20);
    ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/target/poses", 1);
    ros::Subscriber subscriber = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom", 1, [&lock, &pose](nav_msgs::Odometry::ConstPtr odom){
            wait4lock(lock); pose.pose = odom->pose.pose; release(lock);
        }
    );
    ros::Timer tracker = nh.createTimer(
        ros::Duration(1. / fps),
        [&lock, &path, &pose, &fps, &publisher]
        (const ros::TimerEvent&){
            wait4lock(lock); path.poses.push_back(pose); release(lock);
            if(path.poses.size() > fps) path.poses.erase(path.poses.begin());
            publisher.publish(path);
        }
    );
    return ros::spin(), 0;
}
