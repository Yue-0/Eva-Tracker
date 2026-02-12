/* @Author YueLin */

#include <string>

#include <tf/tf.h>
#include <ros/time.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "predictor/bezier.hpp"

namespace eva_tracker
{
    class Tracker
    {
        private:
            ros::Time stamp;
            Eigen::Vector3f point;
            std::string name, world;
            tf::Transform transform;
            Eigen::Quaternionf quaternion;
            tf::TransformBroadcaster broad;
            nav_msgs::Path path, target, observation;
        
        private:
            Bezier* bezier;
            Eigen::Matrix3Xf buffer;
            Eigen::Vector3f position;
            int index = 0, buffers = 0;
        
        private:
            int samples, memory, step, observations;
        
        public:
            Tracker(Bezier* b, std::string frame, std::string tracking, 
                    int dps, int fps, int sample, double interval);
        
        public:
            void filter(const ros::Time& time);
            void prediction(nav_msgs::Path::ConstPtr ctrl);
            bool push(const Eigen::Vector3f& obs, bool refind);
            float broadcast(const ros::Time& time, float yaw);
            nav_msgs::Path* trajectory(const ros::Time& time);
            void localization(nav_msgs::Odometry::ConstPtr odom);
            nav_msgs::Path* update(const ros::Time& time, float yaw);
            nav_msgs::Odometry odom(const std::string& frame, 
                                    const std::string& child) const;
            bool push(){--buffers; return push(Eigen::Vector3f::Zero(), false);}
    };

    nav_msgs::Path FoV(const std::string& frame, 
                       const double distance, 
                       const double alpha, 
                       const double beta);
}
