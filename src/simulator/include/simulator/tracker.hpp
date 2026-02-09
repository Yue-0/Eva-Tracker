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
            Tracker(std::string map, double fps, int sample, double interval);
            void update(geometry_msgs::PoseStamped);
    };
}
