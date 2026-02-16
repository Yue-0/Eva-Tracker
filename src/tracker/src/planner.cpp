/* @Author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

#include "quadrotor_msgs/PositionCommand.h"

#include "planner/plan.hpp"
#include "planner/optim.hpp"

const double PI = std::acos(-1);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Eva-Tracker");
    ros::NodeHandle nh("~"); ros::Time::init();
    ros::MultiThreadedSpinner spinner(nh.param("num_threads", 2));

    /* Frames */
    std::string world, frame;
    nh.getParam("map/frame", world);
    nh.getParam("tracker/frame", frame);

    /* Hyperparmeters */
    const double expansion = 0.5 * nh.param("tracker/size", 0.25);
    const double hz = nh.param("tracker/hz", 50);
    const double dt = 1 / hz;

    /* State of the tracker */
    Eigen::Matrix4Xd states[2] = {
        Eigen::Matrix4Xd::Zero(4, 3),
        Eigen::Matrix4Xd::Zero(4, 3)
    };
    
    /* Initialize map */
    eva_tracker::Map map(
        nh.param("map/length", 1e1),
        nh.param("map/width", 10.0),
        nh.param("map/height", 2.5),
        nh.param("map/resolution", 0.05)
    );

    /* Initialize FoV-ESDF */
    eva_tracker::ESDF fov(
        nh.param("alpha", 65.) * PI / 180,
        nh.param("beta", 40.) * PI / 180, 
        nh.param("D", 5.),
        nh.param("map/resolution", 0.05)
    );

    /* Initialize RC-ESDF */
    eva_tracker::ESDF robot(
        Eigen::Vector3d(
            2 * nh.param("tracker/size", 0.25),
            2 * nh.param("tracker/size", 0.25),
            2 * nh.param("tracker/size", 0.25)
        ),
        nh.param("map/resolution", 0.05),
        nh.param("tracker/expansion", 2.5)
    );

    /* Initialize trajectories */
    eva_tracker::Bezier bezier(
        1 + 2 * nh.param("m", 2), 
        nh.param("tau", 0.5)
    );
    eva_tracker::Minco<4> minco(nh.param("s", 3));
    eva_tracker::Trajectory trajectory(minco.order());

    /* Initialize path generator */
    const double distance = fov.argmax().x();
    eva_tracker::PathPlanner planner(
        &map, distance, nh.param("delta_theta", 1.) * PI / 180.
    );
    ROS_INFO("Optimial observation distance: %f", distance);

    /* Initialize trajectory optimizer */
    eva_tracker::Optimizer optimizer(
        nh.param("ki", 4),
        nh.param("lbfgs/max_iter", 100),
        &minco, &bezier, &robot, &fov, bezier.time,
        nh.param("lbfgs/delta", 1e-5),
        nh.param("lbfgs/min_step", 1e-32),
        nh.param("lbfgs/memory_size", 0x100),
        nh.param("tracker/max_vel_xy", 1.5),
        nh.param("tracker/max_vel_z", 0.1),
        nh.param("tracker/max_vel_yaw", 1.),
        nh.param("tracker/max_acc_xy", 3.),
        nh.param("tracker/max_acc_z", 0.1),
        nh.param("tracker/max_acc_yaw", 1.5),
        nh.param("lambda_p", 1.),
        nh.param("lambda_o", 1.),
        nh.param("lambda_d", 1.),
        nh.param("lambda_a", 1.),
        nh.param("gamma", 1.),
        nh.param("lbfgs/weight", 1.)
    );

    /* Point cloud */
    pcl::PointCloud<pcl::PointXYZ> scan;

    /* Message */
    nav_msgs::Path plan;
    plan.header.frame_id = world;

    /* Wait for transform */
    Eigen::Vector3d target[2];
    (target + 1)->setZero();
    tf::StampedTransform transform;
    tf::TransformListener listener;
    while(!listener.canTransform(world, frame, ros::Time(0)))
    {
        if((target + 1)->x() >= 1)
            ROS_WARN(
                "[%ds] Wait for transform...", 
                static_cast<int>((target + 1)->x())
            );
        listener.waitForTransform(
            world, frame, ros::Time(0), ros::Duration(1)
        );
        (target + 1)->x() += 1;
    }
    ROS_INFO("Transform OK.");
    (target + 1)->setZero();

    /* Publishers */
    ros::Publisher visualizer = nh.advertise<nav_msgs::Path>(
        "/tracker/plan", 1
    );
    ros::Publisher mapper = nh.advertise<sensor_msgs::PointCloud2>(
        "/tracker/map", 1
    );
    ros::Publisher publisher = nh.advertise<quadrotor_msgs::PositionCommand>(
        "/tracker/cmd", 1
    );

    /* Subscribe the pose of the tracker */
    bool ok = false;
    ros::Subscriber localization = nh.subscribe<nav_msgs::Odometry>(
        "/tracker/odom", 1, 
        [&ok, &states](nav_msgs::Odometry::ConstPtr odom){
            ok = true;
            states->col(0) << odom->pose.pose.position.x,
                              odom->pose.pose.position.y,
                              odom->pose.pose.position.z,
                              tf::getYaw(odom->pose.pose.orientation);
        }
    );

    /* Subscribe the position of the target */
    (target + 1)->x() = false;
    ros::Subscriber perception = nh.subscribe<nav_msgs::Odometry>(
        "/target/odom", 1, [&target](nav_msgs::Odometry::ConstPtr odom){
            target->x() = odom->pose.pose.position.x;
            target->y() = odom->pose.pose.position.y;
            target->z() = odom->pose.pose.position.z;
            (target + 1)->y() = odom->twist.twist.linear.y;
            (target + 1)->z() = odom->twist.twist.linear.z;
            (target + 1)->x() = true;
        }
    );

    /* Start and stop */
    bool start = false;
    ros::Subscriber takeoff = nh.subscribe<geometry_msgs::PoseStamped>(
        "/triger", 1, 
        [&start](geometry_msgs::PoseStamped::ConstPtr msg){start = true;}
    );
    ros::Subscriber land = nh.subscribe<geometry_msgs::PoseStamped>(
        "/back_trigger", 1, 
        [&start](geometry_msgs::PoseStamped::ConstPtr msg){start = false;}
    );

    /* Subscribe point cloud from LiDAR */
    ros::Subscriber lidar = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tracker/lidar", 1, [
            &ok, &scan, &states, &map, &target,
            &expansion, &distance, &world, &mapper
        ](sensor_msgs::PointCloud2::ConstPtr msg){
            if(!ok) return;
            
            /* Update map */
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);
            scan = map.update(
                cloud, states->col(0).head(3), target, expansion, distance * 3
            );

            /* Publish point cloud */
            sensor_msgs::PointCloud2 message;
            pcl::toROSMsg(map.map(), message);
            message.header.frame_id = world;
            mapper.publish(message);
        }
    );

    /* Subscribe trajectory prediction result */
    ros::Subscriber tracking = nh.subscribe<nav_msgs::Path>(
        "/target/bezier", 1,  [
            &ok, &bezier, &planner, &map, &states, &optimizer,
            &trajectory, &scan, &plan, &visualizer
        ](nav_msgs::Path::ConstPtr ctrl){
            if(!ok) return;

            /* Update bezier curve */
            int num = ctrl->poses.size();
            for(int p = 0; p < num; p++)
            {
                bezier.control(0, p) = ctrl->poses[p].pose.position.x;
                bezier.control(1, p) = ctrl->poses[p].pose.position.y;
                bezier.control(2, p) = ctrl->poses[p].pose.position.z;
            }
            (states + 1)->col(1).head(3) = bezier.derivative(bezier.duration);

            /* Initial path genaration */
            ros::Time time = ros::Time::now();
            Eigen::Matrix4Xd path = planner.plan(
                states->col(0), bezier.trajectory()
            );
            double t = (ros::Time::now() - time).toSec();

            /* Trajectory optimization */
            if(optimizer.setup(path, states, &scan))
            {
                ROS_DEBUG("Generated new path.\tDuration: %fs", t);
                t = ros::Time::now().toSec();
                optimizer.optimize(&trajectory);
                t = ros::Time::now().toSec() - t;
                ROS_DEBUG("Optimization succeeded.\tDuration: %fs\n", t);
            }
            else
            {
                ROS_DEBUG("Planning failed."); 
                return;
            }

            /* Publish trajectory */
            plan.poses.clear();
            plan.header.stamp = time;
            double duration = trajectory.duration();
            for(double t = 0; t < duration; t += 1e-1)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan.header.stamp;
                Eigen::VectorXd pos = trajectory.pos(t);
                pose.header.frame_id = plan.header.frame_id;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos.w());
                pose.pose.position.x = pos.x();
                pose.pose.position.y = pos.y();
                pose.pose.position.z = pos.z();
                plan.poses.push_back(pose);
            }
            visualizer.publish(plan);
        }
    );

    /* Velocity controller */
    ros::Timer controller = nh.createTimer(ros::Duration(dt), [
        &ok, &start, &plan, &planner, &trajectory, &states, &publisher
    ](const ros::TimerEvent&){
        if(!(ok && start)) return;

        /* Initialize message */
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = plan.header.frame_id;
        cmd.trajectory_flag = quadrotor_msgs
                            ::PositionCommand
                            ::TRAJECTORY_STATUS_READY;

        /* Get state */
        Eigen::Vector4d pos, vel, acc, jerk, now = states->col(0);
        double t = (cmd.header.stamp - plan.header.stamp).toSec();
        if(t > trajectory.duration() || t < 0)
        {
            pos = now;
            vel.setZero();
            acc.setZero();
            jerk.setZero();
        }
        else
        {
            pos = trajectory.pos(t);
            vel = trajectory.vel(t);
            acc = trajectory.acc(t);
            jerk = trajectory.jerk(t);

            // if(!planner.visible(pos.head(3), now.head(3)))
            // {
            //     pos = now;
            //     vel.setZero();
            //     acc.setZero();
            //     jerk.setZero();
            // }
        }
        states->col(1) = vel;

        /* Publish control command */
        cmd.yaw = pos.w();
        cmd.yaw_dot = vel.w();
        cmd.position.x = pos.x();
        cmd.position.y = pos.y();
        cmd.position.z = pos.z();
        cmd.velocity.x = vel.x();
        cmd.velocity.y = vel.y();
        cmd.velocity.z = vel.z();
        cmd.acceleration.x = acc.x();
        cmd.acceleration.y = acc.y();
        cmd.acceleration.z = acc.z();
        if(cmd.yaw > PI || cmd.yaw <= -PI)
            cmd.yaw += 2 * std::floor(0.5 - cmd.yaw / (2 * PI)) * PI;
        cmd.jerk.x = jerk.x();
        cmd.jerk.y = jerk.y();
        cmd.jerk.z = jerk.z();
        publisher.publish(cmd);
    });

    /* Main loop */
    return spinner.spin(), 0;
}
