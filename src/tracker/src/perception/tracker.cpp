/* @Author YueLin */

#include <cmath>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include "perception/tracker.hpp"

const double PI = std::acos(-1);

namespace eva_tracker
{
    Tracker::Tracker(Bezier* b, std::string frame, std::string tracking, 
                     int dps, int fps, int sample, double interval):
        /* Frames */
        name(tracking), world(frame),

        /* Bezier curve */
        bezier(b), 
        
        /* Hyperparameters */
        samples(sample),
        memory(dps / fps),
        step(std::round(fps * interval)),
        observations(std::round(fps * sample * interval) + 1)

    {
        buffer = Eigen::Matrix3Xf::Zero(3, dps / fps);
        observation.header.frame_id = frame;
        target.poses.resize(sample + 1);
        target.header.frame_id = frame;
        path.header.frame_id = frame;
        position.setZero();
    }

    void Tracker::filter(ros::Time time)
    {
        if(!buffers)
            position = (*bezier)[(time - stamp).toSec()].cast<float>();
        else
        {
            position = buffer.rowwise().sum() / buffers;
            buffers = 0;
        }
    }

    bool Tracker::push(Eigen::Vector3f obs, bool refind)
    {
        if(refind)
        {
            buffers = 1;
            observation.poses.clear();
        }
        else ++buffers;
        buffer.col(index++ % memory) = point 
                                     + quaternion.toRotationMatrix() * obs;
        return !(index *= !(index == memory));
    }

    float Tracker::broadcast(ros::Time time, float yaw)
    {
        const float w = quaternion.w();
        const float x = quaternion.x();
        const float y = quaternion.y();
        const float z = quaternion.z();
        yaw += std::atan2(2 * (w * z - x * y), 1 - 2 * (y * y + z * z));
        transform.setRotation(tf::createQuaternionFromYaw(yaw));
        transform.setOrigin(tf::Vector3(position[0], position[1], position[2]));
        broad.sendTransform(tf::StampedTransform(transform, time, world, name));
        return yaw;
    }

    nav_msgs::Path* Tracker::trajectory(ros::Time time)
    {
        target.header.stamp = time;
        if(!path.poses.empty())
        {
            int n = observation.poses.size();
            observation.poses.push_back(path.poses.back());
            if(n > observations)
                observation.poses.erase(observation.poses.begin());
            while(++n <= observations)
                observation.poses.push_back(observation.poses.back());
            for(int p = observations, t = samples; p >= 0; p -= step)
                target.poses[t--] = observation.poses[p];   
        }
        return &target;
    }

    nav_msgs::Path* Tracker::update(ros::Time time, float yaw)
    {
        path.header.stamp = time;
        if(!path.poses.empty())
        {
            geometry_msgs::Point p = path.poses.back().pose.position;
            if(std::fabs(position[0] - p.x) < 1e-2 &&
               std::fabs(position[1] - p.y) < 1e-2 &&
               std::fabs(position[2] - p.z) < 1e-2) return &path;
        }
        geometry_msgs::PoseStamped ps;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        ps.pose.position.x = position[0];
        ps.pose.position.y = position[1];
        // ps.pose.position.z = position[2];
        ps.pose.position.z = 1.2;  // FIXME
        ps.header = path.header;
        path.poses.push_back(ps);
        return &path;
    }

    void Tracker::prediction(nav_msgs::Path::ConstPtr ctrl)
    {
        stamp = ctrl->header.stamp;
        int num = ctrl->poses.size();
        for(int p = 0; p < num; p++)
        {
            bezier->control(0, p) = ctrl->poses[p].pose.position.x;
            bezier->control(1, p) = ctrl->poses[p].pose.position.y;
            bezier->control(2, p) = ctrl->poses[p].pose.position.z;
        }
    }

    void Tracker::localization(nav_msgs::Odometry::ConstPtr odom)
    {
        point[0] = odom->pose.pose.position.x;
        point[1] = odom->pose.pose.position.y;
        point[2] = odom->pose.pose.position.z;
        quaternion.w() = odom->pose.pose.orientation.w;
        quaternion.x() = odom->pose.pose.orientation.x;
        quaternion.y() = odom->pose.pose.orientation.y;
        quaternion.z() = odom->pose.pose.orientation.z;
    }

    nav_msgs::Odometry Tracker::odom(std::string frame, std::string child)
    {
        nav_msgs::Odometry odom;
        odom.child_frame_id = child;
        odom.header.frame_id = frame;
        odom.pose.pose = path.poses.back().pose;
        return odom;
    }

    nav_msgs::Path FoV(std::string frame,
                       double distance, 
                       double alpha,
                       double beta)
    {
        const double y = distance * std::tan(alpha * PI / 360.);
        const double z = distance * std::tan(beta * PI / 360.);
        nav_msgs::Path fov; fov.header.frame_id = frame;
        geometry_msgs::PoseStamped poses[5];
        poses[4].header.frame_id = frame;
        for(int p = 0; p < 4; p++)
        {
            poses[p].pose.position.x = distance;
            poses[p].pose.position.y = y * (p & 1? 1: -1);
            poses[p].pose.position.z = z * (p >> 1? 1: -1);
            poses[p].header.frame_id = fov.header.frame_id;
            fov.poses.push_back(poses[4]);
            fov.poses.push_back(poses[p]);
        }
        poses[2].pose.position.y *= -1;
        poses[3].pose.position.y *= -1;
        for(int p = 4; p; fov.poses.push_back(poses[--p]));
        fov.poses.push_back(poses[3]);
        return fov;
    }
}
