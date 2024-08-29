/* @Author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

#include "searcher/search.hpp"
#include "optimizer/optim.hpp"

#define NODE "Eva-Tracker"
#define ROS_INIT(node) ros::init(argc, argv, node)

#define unlock(lock) *lock = false
#define wait4lock(lock) while(*lock) ros::Duration(1e-4).sleep(); *lock = true

#define transform(from, to, listener, xo, yo, zo, sine, cosine) do{\
    tf::StampedTransform st;\
    listener.lookupTransform(\
        from, to, ros::Time(0), st\
    ); double theta = tf::getYaw(st.getRotation());\
    sine = std::sin(theta); cosine = std::cos(theta);\
    xo = st.getOrigin().x(); yo = st.getOrigin().y(); zo = st.getOrigin().z();\
} while(false)

#define transfer(point, xo, yo, zo, sin, cos) do{\
    double x = point.x - xo;\
    double y = point.y - yo;\
    point.x = x * cos + y * sin;\
    point.y = y * cos - x * sin;\
    point.z -= zo;\
} while(false)

typedef nav_msgs::Path Trajectory;
typedef geometry_msgs::Twist Velocity;
typedef sensor_msgs::PointCloud2 PointCloud;

const double PI = std::acos(-1);

int main(int argc, char* argv[])
{
    ROS_INIT(NODE);
    ros::Time::init();
    ros::NodeHandle nh("~");

    /* Frames */
    std::string world_frame, robot_frame, lidar_frame, map_frame;
    nh.getParam("tracker/frame", robot_frame);
    nh.getParam("tracker/lidar", lidar_frame);
    nh.getParam("tracker/map", map_frame);
    nh.getParam("map/frame", world_frame);

    /* Hyperparmeters */
    const double expansion = 1.5 * nh.param("tracker/size", 0.25);
    const double hz = nh.param("fps", 1e1);
    const double dt = 1 / hz;

    /* Initialize local map */
    eva_tracker::Map map(
        2 * nh.param("map/length", 5.),
        2 * nh.param("map/width", 5.),
        2 * nh.param("map/height", 1.),
        nh.param("map/resolution", 0.05)
    );

    /* Initialize FOV-ESDF */
    eva_tracker::ESDF fov(
        nh.param("camera/alpha", 65.) * PI / 180,
        nh.param("camera/beta", 40.) * PI / 180, 
        nh.param("camera/distance", 5.),
        nh.param("map/resolution", 0.05)
    );

    /* Initialize RC-ESDF */
    eva_tracker::ESDF robot(
        eva_tracker::Point<double>(
            2 * nh.param("tracker/size", 0.25),
            2 * nh.param("tracker/size", 0.25),
            2 * nh.param("tracker/size", 0.25)
        ),
        nh.param("map/resolution", 0.05),
        nh.param("tracker/expansion", 1.5)
    );

    /* Initialize trajectory optimizer */
    eva_tracker::Optimizer optimizer(
        &robot, &fov, 
        nh.param("lambda/max_iter", 100), dt,
        nh.param("tracker/max_vel", 3.),
        nh.param("tracker/max_omega", 2.),
        nh.param("tracker/max_acc", 2.),
        nh.param("tracker/max_alpha", 1.),
        nh.param("lambda/smoothness", 1.),
        nh.param("lambda/feasibility", 1.),
        nh.param("lambda/angle", 1.),
        nh.param("lambda/safety", 1.),
        nh.param("lambda/visibility", 1.),
        nh.param("lambda/occlusion", 1.)
    );

    /* Initialize path searcher */
    const double xb = fov.argmax().x;
    ROS_INFO("The best observation distance is %.2fm", xb);
    eva_tracker::PathSearcher searcher(
        nh.param("opt/timeout", 0.1), xb,
        2 * nh.param("map/length", 5.),
        2 * nh.param("map/width", 5.),
        2 * nh.param("map/height", 2.),
        nh.param("camera/tolerance", 0.5),
        nh.param("map/resolution", 0.05)
    );

    /* Variables */
    eva_tracker::BSpline bs;
    eva_tracker::Path target;
    eva_tracker::Velocity vel;
    eva_tracker::Path velocity;
    eva_tracker::Acceleration acc;
    eva_tracker::PointCloud cloud;
    tf::TransformListener listener;
    const eva_tracker::Point<double> zero(0, 0, 0, 0);

    /* Messages */
    Trajectory plan;
    Velocity control;
    plan.header.frame_id = world_frame;
    
    /* Locks */
    bool locks[4];
    bool* target_lock = locks;
    bool* map_lock = locks + 1;
    bool* cloud_lock = locks + 2;
    bool* curve_lock = locks + 3;
    memset(locks, false, sizeof(locks));

    /* Wait for transforms */
    listener.waitForTransform(
        map_frame, robot_frame, ros::Time(0), ros::Duration(10)
    );
    listener.waitForTransform(
        map_frame, world_frame, ros::Time(0), ros::Duration(10)
    );

    /* Publishers */
    ros::Publisher publisher = nh.advertise<Velocity>("/tracker/cmd_vel", 1);
    ros::Publisher trajectory = nh.advertise<Trajectory>("/tracker/plan", 1);

    /* Subscription trajectory prediction result */
    ros::Subscriber predictor = nh.subscribe<Trajectory>(
        "/target/predict", 1,  [
            &target_lock, &listener, &map_frame, &world_frame, &target
        ](Trajectory::ConstPtr path){
            /* World frame -> Map frame */
            double x0, y0, z0, sin, cos;
            wait4lock(target_lock); target.clear();
            transform(world_frame, map_frame, listener, x0, y0, z0, sin, cos);
            for(geometry_msgs::PoseStamped pose: path->poses)
            {
                double x = pose.pose.position.x - x0;
                double y = pose.pose.position.y - y0;
                target.push_back(eva_tracker::Point<double>(
                    x * cos + y * sin, y * cos - x * sin,
                    pose.pose.position.z - z0
                ));  // Map frame
            }
            unlock(target_lock);
        }
    );

    /* Subscription point cloud */
    ros::Subscriber lidar = nh.subscribe<PointCloud>(
        "/tracker/lidar", 1, [
            &map_lock, &cloud_lock,
            &map, &expansion, &cloud,
            &listener, &lidar_frame, &world_frame
        ](PointCloud::ConstPtr message){

            /* Update map */
            eva_tracker::PointCloud pc;
            pcl::fromROSMsg(*message, pc);  // Lidar frame
            wait4lock(map_lock); map.update(pc, expansion); unlock(map_lock);

            /* Lidar frame -> World frame */
            const int num = pc.size();
            double x0, y0, z0, sin, cos;
            wait4lock(cloud_lock); cloud = pc;
            transform(lidar_frame, world_frame, listener, x0, y0, z0, sin, cos);
            for(int p = 0; p < num; p++)
                transfer(cloud[p], x0, y0, z0, sin, cos);
            unlock(cloud_lock);
        }
    );

    /* Trajectory planning */
    ros::Timer planner = nh.createTimer(
        ros::Duration(1 / nh.param("planning/hz", 1e1)), [
            &target_lock, &map_lock,
            &searcher, &map, &target,
            &world_frame, &map_frame, &listener,
            &cloud_lock, &curve_lock, &bs, &vel, &acc, &dt, &hz,
            &optimizer, &cloud, &velocity, &plan, &trajectory
        ](const ros::TimerEvent&){
            
            /* Path planning */
            wait4lock(target_lock); wait4lock(map_lock);
            eva_tracker::Path path = searcher.plan(map, target);  // Map frame
            eva_tracker::Path prediction = target;
            unlock(map_lock);

            /* Map frame -> World frame */
            int len = path.size();
            tf::StampedTransform st;
            listener.lookupTransform(
                map_frame, world_frame, ros::Time(0), st
            );
            tf::Vector3& origin = st.getOrigin();
            double theta = tf::getYaw(st.getRotation());
            double sin = std::sin(theta), cos = std::cos(theta);
            double x0 = origin.x(), y0 = origin.y(), z0 = origin.z();
            for(int t = 0; t < len; t++)
            {
                path[t].yaw -= theta;
                transfer(path[t], x0, y0, z0, sin, cos);
                if(t) transfer(prediction[t - 1], x0, y0, z0, sin, cos);
            }
            unlock(target_lock);

            /* If no target */
            if(len <= 0) return;

            /* Create B-spline and optimize trajectory */
            wait4lock(cloud_lock);
            bs.create(path, vel, acc, dt);
            bs = optimizer.optimize(bs, &cloud, &prediction);  // World frame
            unlock(cloud_lock);
            
            /* Get velocity */
            path = bs.trajectory();
            wait4lock(curve_lock);
            len = path.size();
            velocity.clear();
            for(int t = 1; t < len; t++)
                velocity.push_back(hz * (path[t] - path[t - 1]));
            std::reverse(velocity.begin(), velocity.end());
            unlock(curve_lock);

            /* Publish planned trajectory */
            plan.poses.clear();
            for(eva_tracker::Point<double> point: path)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;
                plan.poses.push_back(pose);
            }
            trajectory.publish(plan);
        }
    );

    /* Velocity controller */
    ros::Timer controller = nh.createTimer(
        ros::Duration(dt), [
            &curve_lock, &velocity, &zero,
            &listener, &world_frame, &robot_frame,
            &acc, &vel, &hz, &publisher
        ](const ros::TimerEvent&){

            /* Initialize */
            Velocity cmd;
            cmd.linear.x = cmd.linear.y = cmd.linear.z = 0;
            cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

            /* Bounds Checking */
            wait4lock(curve_lock);
            if(!velocity.size())
            {
                unlock(curve_lock);
                if(vel != zero)
                {
                    publisher.publish(cmd);
                    acc = -hz * vel; vel = zero;
                }
                return;
            }
            
            /* World frame -> Robot frame */
            tf::StampedTransform st;
            listener.lookupTransform(
                world_frame, robot_frame, ros::Time(0), st
            );
            double theta = tf::getYaw(st.getRotation());
            double sin = std::sin(theta), cos = std::cos(theta);

            /* Get velocity */
            eva_tracker::Point<double> v = velocity.back(); velocity.pop_back();
            acc = hz * (v - vel); acc.yaw = hz * (v.yaw - vel.yaw); vel = v;

            /* Publish velocity */
            unlock(curve_lock);
            cmd.linear.x = v.x * cos + v.y * sin;
            cmd.linear.y = v.y * cos - v.x * sin;
            cmd.linear.z = v.z; cmd.angular.z = v.yaw;
            publisher.publish(cmd);
        }
    );

    /* Main loop */
    return ros::spin(), 0;
}
