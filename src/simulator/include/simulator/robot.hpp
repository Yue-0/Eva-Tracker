/* @Author: YueLin */

#pragma once

#include <cmath>
#include <unistd.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

const double PI = std::acos(-1);

namespace sim
{
    struct SO3
    {
        double x, y, z, yaw;
    };
    
    class Robot
    {
        public:
            SO3 pose, vel;
            double length, width, height;
        
        private:
            bool lock = false;
        
        public:
            Robot();
            Robot(SO3, double, double, double);
        
        public:
            void move(double);
            void control(SO3);
            void control(geometry_msgs::Twist::ConstPtr);
            nav_msgs::Odometry msg(std::string&, std::string&);
        
        private:
            void unlock() {lock = false;}
            void wait4lock() {while(lock) usleep(100U); lock = true;}
    };
}
