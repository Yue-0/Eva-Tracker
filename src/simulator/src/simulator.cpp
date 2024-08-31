/* @Author: YueLin */

#include <tf/tf.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"

#include "planner/nav.hpp"
#include "simulator/env.hpp"
#include "simulator/lidar.hpp"

#define ROS_INIT(node) ros::init(argc, argv, node)

#define unlock(lock) lock = false
#define wait4lock(lock) while(lock) ros::Duration(1e-4).sleep(); lock = true

typedef nav_msgs::Odometry Odom;
typedef geometry_msgs::Twist Velocity;
typedef geometry_msgs::PoseStamped Pose;
typedef sensor_msgs::LaserScan LaserScan;
typedef sensor_msgs::PointCloud2 PointCloud;

int main(int argc, char* argv[])
{
    ROS_INIT("simulator/env");
    ros::NodeHandle nh("~");
    ros::MultiThreadedSpinner spinner(nh.param("num_threads", 4));

    /* Initialize environment */
    double size = nh.param("tracker/size", 0.25);
    sim::Environment env(
        nh.param("map/resolution", 5e-2),
        nh.param("map/size_x", 0xF),
        nh.param("map/size_y", 0xF),
        nh.param("map/size_z", 0x2),
        nh.param("target/x", 3.),
        nh.param("target/y", 6.),
        nh.param("target/z", 0.25),
        nh.param("target/yaw", 0.0),
        nh.param("target/width", 0.5),
        nh.param("target/length", 1.0),
        nh.param("target/height", 1.0),
        nh.param("tracker/x", 1.),
        nh.param("tracker/y", 6.),
        nh.param("tracker/z", 1.),
        nh.param("tracker/yaw", 0),
        size, size, size
    );

    /* Generate random map */
    env.map.random(
        env.target.pose.x, env.target.pose.y,
        env.tracker.pose.x, env.tracker.pose.y,
        std::max(std::max(env.target.length, env.target.width), size),
        nh.param("map/seed", 0), nh.param("map/num_obstacles", 0x18)
    );
    std::string frame[3];
    nh.getParam("map/frame", frame[0]);
    PointCloud map = sim::map2msg(env.map, frame[0]);
    ros::Publisher publisher = nh.advertise<PointCloud>("/map", 1);
    env.map.expand(std::max(env.target.length, env.target.width) / 2);
    ros::Timer mapper = nh.createTimer(
        ros::Duration(nh.param("map/publish_time", 1)),
        [&publisher, &map](const ros::TimerEvent&){
            publisher.publish(map);
        }, true
    );

    /* Lidar publishers */
    nh.getParam("target/frame", frame[2]);
    nh.getParam("tracker/frame", frame[1]);
    double time = nh.param("lidar/rate", 1e1);
    double high = nh.param("lidar/height", 1.);
    double height = nh.param("lidar/dh", 1e-1);
    double angle = nh.param("lidar/angle", 1.) * PI / 180;
    double lines = nh.param("lidar/single", false)? high: 0;
    double angles = nh.param("tracker/angle", 60.) * PI / 360;
    int range = std::round(nh.param("lidar/max", 5.) / env.map.resolution);
    int dis = std::round(nh.param("tracker/distance", 5.) / env.map.resolution);
    ros::Publisher laser = nh.advertise<PointCloud>("/tracker/lidar", 1);
    ros::Publisher depth = nh.advertise<PointCloud>("/tracker/depth", 1);
    // ros::Publisher sl = nh.advertise<LaserScan>("/target/lidar", 1);
    ros::Timer lidar = sim::LiDAR(
        nh, frame[1], env.map, env.tracker,
        laser, 1 / time, lines, height, angle, range
    );
    ros::Timer camera = sim::depth(
        nh, frame[1], env.map, env.tracker,
        depth, 1 / time, high, height, angles, angle, dis
    );
    // ros::Timer single = sim::LiDAR(
    //     nh, frame[2], env.map, env.target, sl, 1 / time, angle, range
    // );

    /* Position publishers */
    double times[2] = {
        1 / nh.param("target/rate", 1e1),
        1 / nh.param("tracker/rate", 1e2)
    };
    ros::Publisher target = nh.advertise<Odom>("/target/odom", 1);
    ros::Publisher tracker = nh.advertise<Odom>("/tracker/odom", 1);
    ros::Timer target_odom = nh.createTimer(
        ros::Duration(times[1]),
        [&env, &target, &frame](const ros::TimerEvent&){
            target.publish(env.target.msg(frame[0], frame[2]));
        }
    );
    ros::Timer tracker_odom = nh.createTimer(
        ros::Duration(times[1]),
        [&env, &tracker, &frame](const ros::TimerEvent&){
            tracker.publish(env.tracker.msg(frame[0], frame[1]));
        }
    );
    ros::Timer controller = nh.createTimer(
        ros::Duration(times[1]), [&env, &times](const ros::TimerEvent&){
            if(!env.step(times[1])) ROS_ERROR("Collision!");
        }
    );

    /* Target Navigation */
    double x, y;
    bool navigation = false;
    bool lock[2] = {false, false};
    double err = nh.param("target/ctrl_err", 0.5);
    nav::HybirdAStar planner(
        &env.target, times[0], err,
        nh.param("target/max_acc", 1.),
        nh.param("target/safe_d", 0.5),
        nh.param("target/max_vel", 1.5),
        nh.param("target/max_trun", 30.0),
        nh.param("target/back_penalty", 0.5),
        nh.param("target/turn_penalty", 0.1),
        nh.param("target/search_step", 3.)
    );
    std::vector<nav::Node> path;
    std::vector<std::vector<double>> heuristic;
    ros::Publisher trajectory = nh.advertise<nav_msgs::Path>("/target/plan", 1);
    ros::Subscriber goal = nh.subscribe<Pose>(
        "/move_base_simple/goal", 1, [
            &x, &y, &lock, &heuristic, &planner, &env, &navigation
            ,&path, &trajectory, &frame
        ](Pose::ConstPtr p){
            x = p->pose.position.x; y = p->pose.position.y;
            wait4lock(lock[0]); heuristic = planner.dijkstra(env.map, x, y);
            navigation = true; unlock(lock[0]);
        }
    );
    ros::Timer planning = nh.createTimer(
        ros::Duration(times[0]), [
            &navigation, &err, &env, &x, &y, &heuristic,
            &lock, &path, &planner, &trajectory, &frame
        ](const ros::TimerEvent&){
            wait4lock(lock[0]);
            if(!navigation || err >= std::hypot
               (env.target.pose.x - x, env.target.pose.y - y)){
                if(env.target.vel.x){
                    wait4lock(lock[1]); path.clear(); unlock(lock[1]);
                }
                navigation = false; unlock(lock[0]); return;
            }
            std::vector<nav::Node> p = planner.plan(env.map, heuristic);
            unlock(lock[0]); if(!p.empty())
            {
                wait4lock(lock[1]); path = p;
                trajectory.publish(planner.msg(path, frame[0]));
                path.pop_back(); unlock(lock[1]);
            }
        }
    );
    ros::Timer move = nh.createTimer(
        ros::Duration(times[0]), [
            &path, &planner, &lock, &env
        ](const ros::TimerEvent&){
            wait4lock(lock[1]);
            sim::SO3 vel = planner.control(path);
            unlock(lock[1]); env.target.control(vel);
        }
    );

    /* Subscribers */
    ros::Subscriber velocity = nh.subscribe<Velocity>(
        "/tracker/cmd_vel", 1,
        [&env](Velocity::ConstPtr cmd){env.tracker.control(cmd);}
    );
    // ros::Subscriber velocity_target = nh.subscribe<Velocity>(
    //     "/target/cmd_vel", 1,
    //     [&env](Velocity::ConstPtr cmd){env.target.control(cmd);}
    // );
    ros::Subscriber replay = nh.subscribe<Odom>(
        "/target/odom/replay", 1, [&env](Odom::ConstPtr odom){
            env.target.pose.x = odom->pose.pose.position.x;
            env.target.pose.y = odom->pose.pose.position.y;
            env.target.pose.z = odom->pose.pose.position.z;
            env.target.pose.yaw = tf::getYaw(odom->pose.pose.orientation);
        }
    );
    // ros::Subscriber tracker_replay = nh.subscribe<Odom>(
    //     "/tracker/odom/replay", 1, [&env](Odom::ConstPtr odom){
    //         env.tracker.pose.x = odom->pose.pose.position.x;
    //         env.tracker.pose.y = odom->pose.pose.position.y;
    //         env.tracker.pose.z = odom->pose.pose.position.z;
    //         env.tracker.pose.yaw = tf::getYaw(odom->pose.pose.orientation);
    //     }
    // );

    /* Run */
    return spinner.spin(), 0;
}
