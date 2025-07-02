/* @Author: YueLin */

#include "sensor_msgs/LaserScan.h"

#include "simulator/env.hpp"
#include "simulator/lidar.hpp"
#include "simulator/planner.hpp"
#include "simulator/tracker.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simulator"); ros::NodeHandle nh("~");
    ros::MultiThreadedSpinner spinner(nh.param("num_threads", 4));

    /* Initialize environment */
    double size = nh.param("tracker/size", 0.25);
    simulator::Map map(
        nh.param("map/size_x", 0xF),
        nh.param("map/size_y", 0xF),
        nh.param("map/size_z", 0x2),
        nh.param("map/resolution", 5e-2)
    );
    simulator::Environment env(
        &map,
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
    map.random(
        env.target.pose.x, env.target.pose.y,
        env.tracker.pose.x, env.tracker.pose.y,
        std::max(std::max(env.target.length, env.target.width), size),
        nh.param("map/max_obstacle_size", 1), nh.param("map/seed", 0), 
        nh.param("map/num_obstacles", 0x64)
    );
    std::string frame[3];
    nh.getParam("map/frame", frame[0]);
    ros::Publisher mapper = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    map.expand(std::max(env.target.length, env.target.width) / 2);
    ros::Timer once = nh.createTimer(
        ros::Duration(nh.param("map/publish_time", 1)),
        [&mapper, &map, &frame](const ros::TimerEvent&){
            mapper.publish(simulator::map2msg(map, frame[0]));
        }, true
    );
    map.distance();

    /* Lidar publishers */
    nh.getParam("target/frame", frame[2]);
    nh.getParam("tracker/frame", frame[1]);
    double time = nh.param("lidar/rate", 1e1);
    double high = nh.param("lidar/height", 1.);
    double height = nh.param("lidar/dh", 1e-1);
    double angle = nh.param("lidar/angle", 1.) * PI / 180;
    double lines = !nh.param("lidar/single", false)? high: 0;
    double alpha = nh.param("tracker/alpha", 65.) * PI / 360;
    int range = std::round(nh.param("lidar/max", 5.) / map.resolution);
    // int dis = std::round(nh.param("tracker/distance", 5.) / map.resolution);
    ros::Publisher laser = nh.advertise<sensor_msgs::PointCloud2>(
        "/tracker/lidar", 1
    );
    // ros::Publisher depth = nh.advertise<sensor_msgs::PointCloud2>(
    //     "/tracker/depth", 1
    // );
    // ros::Publisher sl = nh.advertise<sensor_msgs::LaserScan>(
    //     "/target/lidar", 1
    // );
    ros::Timer lidar = simulator::LiDAR(
        nh, frame[0], map, env.tracker,
        laser, 1 / time, lines, height, angle, range
    );
    // ros::Timer camera = simulator::depth(
    //     nh, frame[0], map, env.tracker,
    //     depth, 1 / time, high, height, alpha, angle, dis
    // );
    // ros::Timer single = simulator::LiDAR(
    //     nh, frame[2], map, env.target, sl, 1 / time, angle, range
    // );

    /* Visualize FoV */
    const double x = nh.param("tracker/distance", 3.);
    nav_msgs::Path fov = simulator::FoV(frame[1], x, alpha, nh.param(
        "tracker/beta", 40.
    ) * PI / 360);
    ros::Publisher view = nh.advertise<nav_msgs::Path>("/tracker/fov", 1);

    /* Position publishers */
    double times[2] = {
        1 / nh.param("target/rate", 1e1),
        1 / nh.param("tracker/rate", 1e2)
    };
    ros::Publisher trajectories[3] = {
        nh.advertise<nav_msgs::Path>("/target/poses", 1),
        nh.advertise<nav_msgs::Path>("/target/trajectory", 1),
        nh.advertise<nav_msgs::Path>("/tracker/trajectory", 1)
    };
    ros::Publisher odometry[2] = {
        nh.advertise<nav_msgs::Odometry>("/target/odom", 1),
        nh.advertise<nav_msgs::Odometry>("/tracker/odom", 1)
    };
    ros::Publisher tg = nh.advertise<geometry_msgs::PoseStamped>("/triger", 1);
    ros::Timer target = nh.createTimer(
        ros::Duration(times[1]), 
        [&tg, &env, &frame, &trajectories, &odometry](const ros::TimerEvent&){
            tg.publish(geometry_msgs::PoseStamped());
            nav_msgs::Odometry odom = env.target.msg(frame[0], frame[2]);
            trajectories[1].publish(env.target.trajectoy(frame[0], odom));
            env.target.broadcast(odom, ros::Time::now());
            odom.twist.twist.linear.x = std::max(
                env.target.width, env.target.length
            ) / 2;
            odom.twist.twist.linear.y = odom.twist.twist.linear.x;
            odom.twist.twist.linear.z = env.target.height / 2;
            odometry[0].publish(odom);
        }
    );
    ros::Timer tracker = nh.createTimer(
        ros::Duration(times[1]), [
            &env, &frame, &trajectories, &odometry, &view, &fov
        ](const ros::TimerEvent&){
            nav_msgs::Odometry odom = env.tracker.msg(frame[0], frame[1]);
            trajectories[2].publish(env.tracker.trajectoy(frame[0], odom));
            env.tracker.broadcast(odom, ros::Time::now());
            odometry[1].publish(odom);
            view.publish(fov);
        }
    );
    ros::Timer controller = nh.createTimer(
        ros::Duration(times[1]), 
        [&env, &times](const ros::TimerEvent&){env.step(times[1]);}
    );

    /* SOT */
    simulator::Tracker sot(
        frame[0], 
        nh.param("tracker/fps", 1e1), 
        nh.param("tracker/samples", 10),
        nh.param("tracker/interval", 0.1)
    );
    ros::Timer tracking = nh.createTimer(
        ros::Duration(1. / nh.param("tracker/fps", 1e1)), 
        [&sot, &env, &frame, &trajectories](const ros::TimerEvent&){
            sot.update(env.target.msg(frame[0]));
            trajectories[0].publish(sot.path);
        }
    );

    /* Target Navigation */
    bool lock = false;
    simulator::Planner planner(
        &map, &env.target, times[0],
        nh.param("target/max_vel", 1.5),
        nh.param("target/max_acc", 1.0)
    );
    std::vector<std::pair<double, double>> path;
    ros::Publisher trajectory = nh.advertise<nav_msgs::Path>("/target/plan", 1);
    ros::Subscriber goal = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, [
            &map, &frame, &path, &lock, &planner, &trajectory
        ](const geometry_msgs::PoseStamped::ConstPtr& pose){
            if(pose->pose.position.x <= 0 ||
               pose->pose.position.y <= 0 ||
               pose->pose.position.x > map.size0[simulator::X] ||
               pose->pose.position.y > map.size0[simulator::Y]) {
                ROS_WARN("Invalid goal"); return;
            }
            while(lock) 
                ros::Duration(1e-3).sleep();
            lock = true;
            path = planner.plan(pose->pose.position.x, pose->pose.position.y);
            trajectory.publish(planner.msg(frame[0], path));
            std::reverse(path.begin(), path.end());
            lock = false;
        }
    );
    ros::Timer move = nh.createTimer(ros::Duration(times[0]), [
        &path, &planner, &lock, &env, &trajectory, &frame
    ](const ros::TimerEvent&){
        while(lock) 
            ros::Duration(1e-4).sleep();
        lock = true; 
        const int n = path.size();
        env.target.control(planner.control(path));
        if(n == 3 && path.size() < 3)
            trajectory.publish(planner.msg(frame[0], path));
        lock = false;
    });

    /* Subscribers */
    ros::Subscriber command = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "/tracker/cmd", 1,
        [&env](quadrotor_msgs::PositionCommand::ConstPtr cmd){
            env.tracker.pose.yaw = cmd->yaw;
            env.tracker.vel.x = cmd->velocity.x;
            env.tracker.vel.y = cmd->velocity.y;
            env.tracker.vel.z = cmd->velocity.z;
        }
    );
    bool benchmarking = false;
    ros::Subscriber bag = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom/replay", 1,
        [&benchmarking, &env](nav_msgs::Odometry::ConstPtr odom){
            if(!benchmarking) benchmarking = true;
            env.target.pose.x = odom->pose.pose.position.x;
            env.target.pose.y = odom->pose.pose.position.y;
            env.target.pose.z = odom->pose.pose.position.z;
            env.target.pose.yaw = tf::getYaw(odom->pose.pose.orientation);
        }
    );
    // ros::Subscriber replay = nh.subscribe<nav_msgs::Odometry>(
    //     "/tracker/odom/replay", 1, [&env](nav_msgs::Odometry::ConstPtr odom){
    //         env.tracker.pose.x = odom->pose.pose.position.x;
    //         env.tracker.pose.y = odom->pose.pose.position.y;
    //         env.tracker.pose.z = odom->pose.pose.position.z;
    //         env.tracker.pose.yaw = tf::getYaw(odom->pose.pose.orientation);
    //     }
    // );

    /* Benchmarking */
    ros::Timer benchmark = nh.createTimer(
        ros::Duration(times[0]),
        [&benchmarking, &env, &alpha, &x](const ros::TimerEvent&){
            if(!benchmarking) return;
            
            /* Calculate yaw angle error */
            double ae = env.angle();

            /* Calculate tracking distance */
            double td = env.distance();

            /* Calculate the projected position of the target */
            double xp, yp; env.project(&xp, &yp);

            /* Print metrics */
            if(td < 1)
                ROS_WARN("Too near!");
            else if(ae >= alpha)
                ROS_WARN("Out of FoV!");
            else if(env.occlusion())
                ROS_WARN("Occlusion!");
            else if(td > x * 1.5)
                ROS_WARN("Out of FoV!");
            else
                ROS_INFO("Success tracking!");
            std::cout << "TD: " << ae << "\nAE: " << td 
                      << "\nProjected: " << xp << " " << yp << std::endl;
        }
    );

    /* Run */
    return spinner.spin(), 0;
}
