/* @Author: YueLin */

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"

#include "simulator/env.hpp"
#include "simulator/lidar.hpp"
#include "simulator/planner.hpp"
#include "simulator/tracker.hpp"

std::vector<Eigen::Vector2d> plan(double xg, double yg,
                                  const std::string& frame,
                                  ros::Publisher* publisher,
                                  simulator::Planner* planner)
{
    std::vector<Eigen::Vector2d> path = planner->plan(xg, yg);
    publisher->publish(planner->msg(frame, path));
    std::reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char* argv[])
{
    /* Initialize ROS */
    ros::init(argc, argv, "simulator"); ros::NodeHandle nh("~");
    ros::MultiThreadedSpinner spinner(nh.param("num_threads", 4));

    /* Initialize environment */
    const double size = nh.param("tracker/size", 0.25);
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
        env.target.pose.x(), env.target.pose.y(),
        env.tracker.pose.x(), env.tracker.pose.y(),
        std::max(std::max(env.target.length, env.target.width), size),
        nh.param("map/max_obstacle_size", 1), nh.param("map/seed", 0), 
        nh.param("map/num_obstacles", 0x64)
    );
    std::string frame[3];
    nh.getParam("map/frame", *frame);
    ros::Publisher mapper = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    map.expand(std::max(env.target.length, env.target.width) / 2);
    ros::Timer once = nh.createTimer(
        ros::Duration(nh.param("map/publish_time", 1)),
        [&mapper, &map, &frame](const ros::TimerEvent&){
            mapper.publish(simulator::map2msg(map, *frame));
            ROS_INFO("Map initialized.");
        }, true
    );
    map.distance();

    /* Lidar publishers */
    nh.getParam("target/frame", frame[2]);
    nh.getParam("tracker/frame", frame[1]);
    const double time = nh.param("lidar/rate", 1e1);
    const double high = nh.param("lidar/height", 1.);
    const double height = nh.param("lidar/dh", 1e-1);
    const double angle = nh.param("lidar/angle", 1.) * PI / 180;
    const double lines = !nh.param("lidar/single", false)? high: 0;
    const double alpha = nh.param("tracker/alpha", 65.) * PI / 360;
    const int range = std::round(nh.param("lidar/max", 5.) / map.resolution);
    // const int dis = nh.param("tracker/distance", 3.) / map.resolution;
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
        nh, *frame, laser, map, env.tracker,
        1 / time, lines, height, angle, range
    );
    // ros::Timer camera = simulator::depth(
    //     nh, *frame, depth, map, env.tracker,
    //     1 / time, high, height, alpha, angle, dis
    // );
    // ros::Timer single = simulator::LiDAR(
    //     nh, frame[2], sl, map, env.target, 1 / time, angle, range
    // );

    /* Visualize FoV */
    const double x = nh.param("tracker/distance", 3.);
    const nav_msgs::Path fov = simulator::FoV(
        frame[1], x, alpha, nh.param("tracker/beta", 40.) * PI / 360
    );
    ros::Publisher view = nh.advertise<nav_msgs::Path>("/tracker/fov", 1);

    /* Position publishers */
    const double times[2] = {
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
            nav_msgs::Odometry odom = env.target.msg(*frame, frame[2]);
            trajectories[1].publish(env.target.trajectoy(*frame, odom));
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
            const nav_msgs::Odometry odom = env.tracker.msg(*frame, frame[1]);
            trajectories[2].publish(env.tracker.trajectoy(*frame, odom));
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
        *frame, 
        nh.param("tracker/fps", 1e1), 
        nh.param("tracker/samples", 10),
        nh.param("tracker/interval", 0.1)
    );
    ros::Timer tracking = nh.createTimer(
        ros::Duration(1. / nh.param("tracker/fps", 1e1)), 
        [&sot, &env, &frame, &trajectories](const ros::TimerEvent&){
            sot.update(env.target.msg(*frame));
            trajectories->publish(sot.path);
        }
    );

    /* Target Navigation */
    simulator::Lock lock;
    simulator::Planner planner(
        &map, &env.target, times[0],
        nh.param("target/max_vel", 1.5),
        nh.param("target/max_acc", 1.0),
        nh.param("planner/lambda", 1.0),
        nh.param("planner/past", 3),
        nh.param("planner/memory", 64),
        nh.param("planner/iters", 100),
        nh.param("planner/eps", 1e-6),
        nh.param("planner/steps", 1e20),
        nh.param("planner/delta", 1e-6),
        nh.param("planner/epsilon", 1e-5),
        nh.param("planner/wolfe", 9e-1),
        nh.param("planner/armijo", 1e-4)
    );
    std::vector<Eigen::Vector2d> path;
    ros::Publisher trajectory = nh.advertise<nav_msgs::Path>("/target/plan", 1);
    ros::Subscriber goal = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, [
            &map, &lock, &path, &frame, &trajectory, &planner
        ](const geometry_msgs::PoseStamped::ConstPtr& pose){
            if(pose->pose.position.x <= 0 ||
               pose->pose.position.y <= 0 ||
               pose->pose.position.x > map.size0.x() ||
               pose->pose.position.y > map.size0.y()) ROS_WARN("Invalid goal");
            else
            {
                lock.acquire();
                path = plan(
                    pose->pose.position.x,
                    pose->pose.position.y,
                    *frame, &trajectory, &planner
                );
                lock.release();
            }
        }
    );
    bool automatic = false;
    ros::Subscriber click = nh.subscribe<geometry_msgs::PointStamped>(
        "/clicked_point", 1, 
        [&automatic](const geometry_msgs::PointStamped::ConstPtr&){
            if((automatic = !automatic)) std::srand(std::time(0));
        }
    );
    ros::Timer random = nh.createTimer(ros::Duration(times[0]), [
        &automatic, &path, &map, &lock, &frame, &trajectory, &planner
    ](const ros::TimerEvent&){
        if(automatic && path.empty())
        {
            Eigen::Vector2d p = map.random();
            lock.acquire();
            path = plan(p.x(), p.y(), *frame, &trajectory, &planner);
            lock.release();
        }
    });
    ros::Timer move = nh.createTimer(ros::Duration(times[0]), [
        &path, &planner, &lock, &env, &trajectory, &frame
    ](const ros::TimerEvent&){
        lock.acquire();
        const int n = path.size();
        env.target.control(planner.control(path));
        if(n == 3 && path.size() < 3)
        {
            path.clear();
            trajectory.publish(planner.msg(*frame, path));
        }
        lock.release();
    });

    /* Subscribers */
    ros::Subscriber command = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "/tracker/cmd", 1,
        [&env](quadrotor_msgs::PositionCommand::ConstPtr cmd){
            env.tracker.pose.w() = cmd->yaw;
            env.tracker.vel.x() = cmd->velocity.x;
            env.tracker.vel.y() = cmd->velocity.y;
            env.tracker.vel.z() = cmd->velocity.z;
        }
    );
    bool benchmarking = false;
    ros::Subscriber bag = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom/replay", 1,
        [&benchmarking, &env](nav_msgs::Odometry::ConstPtr odom){
            if(!benchmarking) benchmarking = true;
            env.target.pose << odom->pose.pose.position.x,
                               odom->pose.pose.position.y,
                               odom->pose.pose.position.z,
                               tf::getYaw(odom->pose.pose.orientation);
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
        [&benchmarking, &automatic, &env, &alpha, &x](const ros::TimerEvent&){
            if(benchmarking || automatic)
            {
                /* Calculate yaw angle error */
                const double ae = env.angle();
                const bool out = ae >= alpha;
                if(out)
                    ROS_WARN("AE: %f", ae);
                else
                    ROS_INFO("AE: %f", ae);
                
                /* Calculate tracking distance */
                const double td = env.distance();
                const bool far = td > x * 1.5;
                const bool near = td < 1;
                if(far || near)
                    ROS_WARN("TD: %f", td);
                else
                    ROS_INFO("TD: %f", td);

                /* Print tracking status */
                if(near)
                    ROS_WARN("Too near!");
                else if(out)
                    ROS_WARN("Out of FoV!");
                else if(env.occlusion())
                    ROS_WARN("Occlusion!");
                else if(far)
                    ROS_WARN("Out of FoV!");
                else
                    ROS_INFO("Success tracking!");

                /* Calculate the projected position of the target */
                const Eigen::Vector2d projected = env.project();
                ROS_DEBUG("Projected: %f %f", projected.x(), projected.y());
            }
        }
    );

    /* Run */
    return spinner.spin(), 0;
}
