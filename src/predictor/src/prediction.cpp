/* @Author: YueLin */

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "predictor/predictor.hpp"

#define ROS_INIT(node) ros::init(argc, argv, node)

using namespace trajectory_prediction;

typedef nav_msgs::Path Trajectory;

int main(int argc, char* argv[])
{
    ROS_INIT("predictor");
    ros::NodeHandle nh("~");

    /* Initialize trajectory predictor */
    Points trajectory;
    Trajectory prediction;
    Predictor predictor(
        nh.param("gamma", 0.1),
        nh.param("max_iterations", 0)
    );

    /* Publisher */
    ros::Publisher publisher = nh.advertise<Trajectory>("/target/predict", 1);

    /* Subscriber */
    ros::Subscriber subscriber = nh.subscribe<Trajectory>(
        "/target/poses", 1, [&](Trajectory::ConstPtr path){
            trajectory.clear();
            prediction.poses.clear();
            prediction.header.frame_id = path->header.frame_id;
            for(geometry_msgs::PoseStamped pose: path->poses)
                trajectory.push_back(eva_tracker::Point<double>(
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ));
            for(eva_tracker::Point<double> point: predictor.predict(trajectory))
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = path->header.frame_id;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;
                prediction.poses.push_back(pose);
            }
            publisher.publish(prediction);
        }
    );

    /* Start prediction */
    return ros::spin(), 0;
}
