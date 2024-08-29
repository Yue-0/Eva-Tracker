/* @Author: YueLin */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"

#define TARGET 1
#define TRACKER 0

#define ROS_INIT(node) ros::init(argc, argv, node)

#define callback(robot) [&](nav_msgs::Odometry::ConstPtr odom){\
    transform[robot].setOrigin(tf::Vector3(\
        odom->pose.pose.position.x,\
        odom->pose.pose.position.y,\
        odom->pose.pose.position.z \
    ));\
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q[robot]);\
    transform[robot].setRotation(q[robot]);\
    tb[robot].sendTransform(tf::StampedTransform(\
        transform[robot],\
        ros::Time::now(),\
        odom->header.frame_id, odom->child_frame_id\
    ));\
}

int main(int argc, char* argv[])
{
    /* Initialize */
    tf::Quaternion q[2];
    ROS_INIT("simulator/tf");
    ros::NodeHandle nh("~");
    tf::Transform transform[2];
    tf::TransformBroadcaster tb[2];

    /* Subscribers */
    ros::Subscriber target = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom", 1, callback(TARGET)
    );
    ros::Subscriber tracker = nh.subscribe<nav_msgs::Odometry>(
        "/tracker/odom", 1, callback(TRACKER)
    );

    /* Broadcast */
    return ros::spin(), 0;
}
