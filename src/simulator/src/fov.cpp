/* @Author: YueLin */

#include <cmath>

#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#define ROS_INIT(node) ros::init(argc, argv, node)

int main(int argc, char* argv[])
{
    int p = 0;
    ROS_INIT("fov");
    ros::Time::init();
    nav_msgs::Path path;
    ros::NodeHandle nh("~");
    const double PI = std::acos(-1);
    geometry_msgs::PoseStamped pose[5];
    double x = nh.param("tracker/distance", 3.);
    ros::Rate sleeper(nh.param("tracker/rate", 1e2));
    nh.getParam("tracker/frame", path.header.frame_id);
    nh.getParam("tracker/frame", pose[4].header.frame_id);
    double z = x * std::tan(nh.param("tracker/beta", 40.) * PI / 360);
    double y = x * std::tan(nh.param("tracker/alpha", 65.) * PI / 360);
    while(p < 4)
    {
        pose[p].pose.position.x = x;
        pose[p].pose.position.y = y * (p & 1? 1: -1);
        pose[p].pose.position.z = z * (p >> 1? 1: -1);
        pose[p].header.frame_id = path.header.frame_id;
        path.poses.push_back(pose[4]); path.poses.push_back(pose[p++]);
    }
    pose[3].pose.position.y *= -1; pose[2].pose.position.y *= -1;
    while(p--)
        path.poses.push_back(pose[p]);
    path.poses.push_back(pose[3]);
    ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/tracker/fov", 1);
    while(ros::ok())
    {
        publisher.publish(path);
        sleeper.sleep();
    }
    return 0;
}
