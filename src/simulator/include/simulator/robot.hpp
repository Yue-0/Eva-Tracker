/* @Author: YueLin */

#pragma once

#include <cmath>
#include <unistd.h>

#include <tf/tf.h>
#include <ros/time.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include "quadrotor_msgs/PositionCommand.h"

const double PI = std::acos(-1);

namespace simulator
{
    inline double clip(double rad)
    {
        if(rad > PI || rad <= -PI)
            return 2 * std::floor(0.5 - rad / (2 * PI)) * PI + rad;
        return rad;
    }

    class Lock
    {
        private:
            bool value = false;
        
        public:
            Lock() = default;
        
        private:
            void sleep1ms() const {usleep(1000U);}

        public:
            void release() {value = false;}
            void acquire() {while(value) sleep1ms(); value = true;}
    };
    
    class Robot
    {
        public:
            Eigen::Vector4d pose, vel;
            double length, width, height;
        
        private:
            Lock lock;
            nav_msgs::Path path;
            tf::Transform transform;
            tf::Quaternion quaternion;
            tf::TransformBroadcaster b;
        
        public:
            Robot()
            {
                vel.setZero();
                pose.setZero();
            }

            Robot(const Eigen::Vector4d& position, double l, double w, double h)
            : length(l), width(w), height(h)
            {
                vel.setZero();
                pose = position;
            }
        
        public:
            void move(double dt)
            {
                lock.acquire();
                pose += vel * dt;
                pose.w() = clip(pose.w());
                lock.release();
            }

            void control(const Eigen::Vector4d& velocity)
            {
                lock.acquire();
                vel = velocity;
                lock.release();
            }

            void control(quadrotor_msgs::PositionCommand::ConstPtr cmd)
            {
                lock.acquire();
                vel << cmd->velocity.x, 
                       cmd->velocity.y, 
                       cmd->velocity.z, 
                       cmd->yaw_dot;
                lock.release();
            }

            void broadcast(const nav_msgs::Odometry& odom, const ros::Time& t)
            {
                transform.setOrigin(tf::Vector3(
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z 
                ));
                tf::quaternionMsgToTF(odom.pose.pose.orientation, quaternion);
                transform.setRotation(quaternion);
                b.sendTransform(tf::StampedTransform(
                    transform, t, odom.header.frame_id, odom.child_frame_id
                ));
            }

            geometry_msgs::PoseStamped msg(const std::string& frame) const
            {
                geometry_msgs::PoseStamped ps;
                ps.pose.position.x = pose.x();
                ps.pose.position.y = pose.y();
                ps.pose.position.z = pose.z();
                ps.pose.orientation = tf::createQuaternionMsgFromYaw(pose.w());
                ps.header.frame_id = frame;
                return ps;
            }
            
            nav_msgs::Odometry msg(const std::string& frame, 
                                   const std::string& child)
            {
                nav_msgs::Odometry odom;
                odom.child_frame_id = child;
                odom.header.frame_id = frame;
                odom.twist.twist.linear.x = vel.x();
                odom.twist.twist.linear.y = vel.y();
                odom.twist.twist.linear.z = vel.z();
                odom.pose.pose.position.x = pose.x();
                odom.pose.pose.position.y = pose.y();
                odom.pose.pose.position.z = pose.z();
                odom.twist.twist.angular.z = vel.w();
                odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                    pose.w()
                );
                return odom;
            }

            nav_msgs::Path trajectoy(const std::string& frame,
                                     const nav_msgs::Odometry& odom)
            {
                geometry_msgs::PoseStamped pose;
                path.header.frame_id = frame;
                pose.pose = odom.pose.pose;
                pose.header = odom.header;
                if(!path.poses.empty())
                {
                    geometry_msgs::Point p = path.poses.back().pose.position;
                    if(std::fabs(odom.pose.pose.position.x - p.x) < 1e-2 &&
                       std::fabs(odom.pose.pose.position.y - p.y) < 1e-2 &&
                       std::fabs(odom.pose.pose.position.z - p.z) < 1e-2)
                        return path;
                }
                path.poses.push_back(pose); return path;
            }
    };
}
