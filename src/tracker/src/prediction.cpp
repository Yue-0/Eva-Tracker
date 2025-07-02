/* @Author: YueLin */

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "predictor/predictor.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "predictor");
    ros::NodeHandle nh("~");

    /* Hyperparameters */
    const int m = nh.param("m", 2);
    const double tau = nh.param("tau", 1. / m);

    /* Initialize trajectory predictor */
    nav_msgs::Path prediction;
    Eigen::Matrix3Xd trajectory(3, 1 + m);
    eva_tracker::Bezier bezier(1 + 2 * m, tau);
    eva_tracker::Predictor predictor(&bezier);

    /* Publishers */
    ros::Publisher ctrl = nh.advertise<nav_msgs::Path>("/target/bezier", 1);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/target/predict", 1);

    /* Subscriber */
    ros::Subscriber subscriber = nh.subscribe<nav_msgs::Path>(
        "/target/poses", 1, [&](nav_msgs::Path::ConstPtr path){
            
            /* Extract trajectory */
            for(int p = 0; p <= m; p++)
            {
                trajectory(0, p) = path->poses[p].pose.position.x;
                trajectory(1, p) = path->poses[p].pose.position.y;
                trajectory(2, p) = path->poses[p].pose.position.z;
            }

            /* Predict trajectory */
            predictor.predict(trajectory);

            /* Publish bezier curve */
            prediction.poses.clear();
            prediction.header = path->header;
            geometry_msgs::PoseStamped pose;
            pose.header = path->header;
            for(int p = 0; p <= bezier.n; p++)
            {
                pose.pose.position.x = bezier.control(0, p);
                pose.pose.position.y = bezier.control(1, p);
                pose.pose.position.z = bezier.control(2, p);
                prediction.poses.push_back(pose);
            }
            ctrl.publish(prediction);
            
            /* Publish predicted trajectory */
            prediction.poses.clear();
            for(double t = 0; t <= m * tau; t += 1e-1)
            {
                Eigen::Vector3d point = bezier[t];
                pose.pose.position.x = point(0);
                pose.pose.position.y = point(1);
                pose.pose.position.z = point(2);
                prediction.poses.push_back(pose);
            }
            pub.publish(prediction);
        }
    );

    /* Start prediction */
    return ros::spin(), 0;
}
