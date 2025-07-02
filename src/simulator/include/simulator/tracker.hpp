/* @Author: YueLin */

#include <string>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

namespace simulator
{
    class Tracker
    {
        public:
            nav_msgs::Path path;

        private:
            nav_msgs::Path observation;
            int observations, samples, step;
        
        public:
            Tracker(std::string, double, int, double);
            void update(geometry_msgs::PoseStamped);
    };
}
