/* @Author: YueLin */

#include <chrono>
#include <limits>

#include "nav_msgs/Path.h"

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

#define TIME 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(\
    std::chrono::system_clock::now().time_since_epoch()\
).count

namespace nav
{
    const int VEL = 0, RAD = 1, ACC = 2;

    const double O = 1e-9;
    const double INF = std::numeric_limits<double>::infinity();

    struct Node
    {
        int id, parent;
        double h, g = 0;
        double x, y, yaw;
        double v = 0, w = 0;
    };
    
    class HybirdAStar
    {
        private:
            sim::Robot* car;
            std::vector<Node> nodes;
        
        private:
            double time, error, ds;
            double limit[VEL + RAD + ACC], interval[ACC], penalty[ACC];
        
        public:
            HybirdAStar(sim::Robot*, double, double, double,
                        double, double, double, double, double, int);
            std::vector<Node> plan(sim::Map&, std::vector<std::vector<double>>);
            std::vector<std::vector<double>> dijkstra(sim::Map&, float, float);
            nav_msgs::Path msg(std::vector<Node>&, std::string&);
            sim::SO3 control(std::vector<Node>&);
        
        private:
            double f(Node&);
            std::vector<Node> expand(sim::Map&, Node&);
    };
}
