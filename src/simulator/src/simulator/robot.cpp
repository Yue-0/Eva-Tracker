/* @Author: YueLin */

#include <cmath>

#include <tf/tf.h>

#include "simulator/robot.hpp"

namespace sim
{
    Robot::Robot()
    {
        vel.x = vel.y = vel.z = vel.yaw = 0;
        pose.x = pose.y = pose.z = pose.yaw = 0;
    }

    Robot::Robot(SO3 position, double l, double w, double h)
    {
        width = w;
        length = l;
        height = h;
        pose.x = position.x;
        pose.y = position.y;
        pose.z = position.z;
        pose.yaw = position.yaw;
        vel.x = vel.y = vel.z = vel.yaw = 0;
    }

    void Robot::move(double dt)
    {
        wait4lock();
        double sin, cos;
        pose.z += vel.z * dt;
        if(std::fabs(vel.yaw) <= 1e-6)
        {
            sin = std::sin(pose.yaw);
            cos = std::cos(pose.yaw);
            pose.x += (vel.x * cos - vel.y * sin) * dt;
            pose.y += (vel.x * sin + vel.y * cos) * dt;
        }
        else
        {
            double yaw = pose.yaw + vel.yaw * dt;
            // sin = std::sin(yaw) - std::sin(pose.yaw);
            // cos = std::cos(pose.yaw) - std::cos(yaw);
            // pose.x += (vel.x * sin - vel.y * cos) / vel.yaw;
            // pose.y += (vel.x * cos + vel.y * sin) / vel.yaw;
            sin = std::sin(pose.yaw);
            cos = std::cos(pose.yaw);
            pose.x += (vel.x * cos - vel.y * sin) * dt;
            pose.y += (vel.x * sin + vel.y * cos) * dt;
            while(yaw <= -PI) yaw += 2 * PI;
            while(yaw > PI) yaw -= 2 * PI;
            pose.yaw = yaw;
        }
        unlock();
    }

    void Robot::control(SO3 velocity)
    {
        wait4lock();
        vel.x = velocity.x;
        vel.y = velocity.y;
        vel.z = velocity.z;
        vel.yaw = velocity.yaw;
        unlock();
    }

    void Robot::control(geometry_msgs::Twist::ConstPtr velocity)
    {
        wait4lock();
        vel.x = velocity->linear.x;
        vel.y = velocity->linear.y;
        vel.z = velocity->linear.z;
        vel.yaw = velocity->angular.z;
        unlock();
    }

    nav_msgs::Odometry Robot::msg(std::string& frame, std::string& child)
    {
        nav_msgs::Odometry odom;
        odom.child_frame_id = child;
        odom.header.frame_id = frame;
        odom.twist.twist.linear.x = vel.x;
        odom.twist.twist.linear.y = vel.y;
        odom.twist.twist.linear.z = vel.z;
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = pose.z;
        odom.twist.twist.angular.z = vel.yaw;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.yaw);
        return odom;
    }
}
