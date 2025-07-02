/* @Author: YueLin */

#include "simulator/tracker.hpp"

namespace simulator
{
    Tracker::Tracker(std::string map, double fps, int sample, double interval):
        observations(std::round(fps * sample * interval) + 1),
        samples(sample), step(std::round(fps * interval)){
        observation.header.frame_id = map;
        path.poses.resize(sample + 1);
        path.header.frame_id = map;
    }

    void Tracker::update(geometry_msgs::PoseStamped now)
    {
        int n = observation.poses.size();
        observation.poses.push_back(now);
        if(n > observations)
            observation.poses.erase(observation.poses.begin());
        while(++n <= observations)
            observation.poses.push_back(observation.poses.back());
        for(int p = observations, t = samples; p >= 0; p -= step)
            path.poses[t--] = observation.poses[p];
    }
}

