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
            Tracker(Bezier*, std::string, std::string, int, int, int, double);
        
        public:
            void filter(ros::Time);
            bool push(Eigen::Vector3f, bool);
            float broadcast(ros::Time, float);
            nav_msgs::Path* trajectory(ros::Time);
            nav_msgs::Path* update(ros::Time, float);
            void prediction(nav_msgs::Path::ConstPtr);
            void localization(nav_msgs::Odometry::ConstPtr);
            nav_msgs::Odometry odom(std::string, std::string);
            bool push(){--buffers; return push(Eigen::Vector3f::Zero(), false);}
    };

    nav_msgs::Path FoV(std::string, double, double, double);
}
