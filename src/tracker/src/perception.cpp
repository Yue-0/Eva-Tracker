/* @Author: YueLin */

#include <ros/ros.h>

#include "perception/yolo.hpp"
#include "perception/tracker.hpp"
#include "perception/monoloco.hpp"

#include "tracker/Camera.h"

int main(int argc, char* argv[])
{
    /* Initialize ROS */
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh("~");

    /* Models */
    bool lost = false;
    std::string tracker, target;
    eva_tracker::Bezier bezier(
        1 + 2 * nh.param("m", 10), 
        nh.param("tau", 0.1)
    );
    nh.getParam("monoloco", target);
    nh.getParam("yolo11pose", tracker);
    eva_tracker::MonoLoco monoloco(target);
    eva_tracker::YOLOv11Pose yolo(tracker);
    const bool visualize = nh.param("visualize", true);
    const double threshold = nh.param("threshold", 0.4);

    /* Frames */
    std::string world;
    nh.getParam("frame/world", world);
    nh.getParam("frame/target", target);
    nh.getParam("frame/tracker", tracker);
    
    /* Tracker */
    eva_tracker::Tracker server(
        &bezier, world, target, 
        nh.param("dps", 30), nh.param("fps", 30), 
        nh.param("m", 10), nh.param("tau", 0.1)
    );

    /* Publishers */
    ros::Publisher view = nh.advertise<nav_msgs::Path>(
        "/tracker/fov", 1
    );
    ros::Publisher tracking = nh.advertise<nav_msgs::Path>(
        "/target/poses", 1
    );
    ros::Publisher trajectory = nh.advertise<nav_msgs::Path>(
        "/target/trajectory", 1
    );
    ros::Publisher detector = nh.advertise<nav_msgs::Odometry>(
        "/target/odom", 1
    );
    ros::Publisher visualizer = nh.advertise<sensor_msgs::Image>(
        "/tracker/fpv", 1
    );

    /* Subscribe the pose of the tracker */
    bool ok = false;
    nav_msgs::Path fov = eva_tracker::FoV(
        tracker, nh.param("camera/distance", 3.0),
        nh.param("camera/alpha", 69.4), nh.param("camera/beta", 42.5)
    );
    ros::Subscriber localization = nh.subscribe<nav_msgs::Odometry>(
        "/tracker/odom", 1, 
        [&server, &view, &fov, &ok](nav_msgs::Odometry::ConstPtr odom){
            server.localization(odom); view.publish(fov); ok = true;
        }
    );

    /* Subscribe the predicted position of the target */
    ros::Subscriber prediction = nh.subscribe<nav_msgs::Path>(
        "/target/bezier", 1, [&lost, &server](nav_msgs::Path::ConstPtr ctrl){
            if(!lost) server.prediction(ctrl);
        }
    );

    /* Main loop */
    ros::Subscriber camera = nh.subscribe<tracker::Camera>(
        "/tracker/camera", 1, [&](tracker::Camera::ConstPtr frames){
            if(!ok)
                return;
            bool publish;
            ros::Time time = ros::Time::now();

            /* Get RGB image */
            cv::Mat color(
                frames->color.height, frames->color.width,
                CV_8UC3, (void*)frames->color.data.data()
            );

            /* Keypoints detection */
            std::vector<eva_tracker::Person> detections = yolo.detect(
                color, threshold, true, 0
            );

            /* Keypoints -> 3D pose */
            if(detections.empty())
            {
                lost = true;
                publish = server.push();
            }
            else
            {
                cv::Mat depth(
                    frames->depth.height, frames->depth.width,
                    CV_16UC1, (void*)frames->depth.data.data()
                );
                depth.convertTo(depth, CV_32FC1, frames->scale);
                monoloco.process(
                    detections.front().skeleton,
                    frames->fx, frames->fy,
                    frames->cx, frames->cy
                );
                publish = server.push(yolo.solve(
                    depth, detections.front(),
                    frames->fx, frames->fy,
                    frames->cx, frames->cy
                ), lost);
                lost = false;
            }

            /* Publish messages */
            if(publish)
            {
                server.filter(time);
                trajectory.publish(*(server.update(
                    time, server.broadcast(time, monoloco.yaw)
                )));
                tracking.publish(*(server.trajectory(time)));
                nav_msgs::Odometry odom = server.odom(world, target);
                odom.twist.twist.linear.z = yolo.height * 1.5;
                odom.twist.twist.linear.y = yolo.width * 1.5;
                odom.twist.twist.linear.x = yolo.width * 1.5;
                detector.publish(odom);
            }

            /* Visualize detection results */
            if(visualize)
            {
                sensor_msgs::Image image = frames->color;
                yolo.visualize(color, detections, true, true, true);
                image.data.assign(color.datastart, color.dataend);
                visualizer.publish(image);
            }
        }
    );

    /* Spin */
    return ros::spin(), 0;
}
