/* @Author: YueLin */

#include <cmath>

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace sim
{
    class Environment
    {
        public:
            Map map;
            Robot target;
            Robot tracker;
        
        public:
            bool step(double);
            Environment(
                double, double, double, double, double, double,
                double, double, double, double, double, double,
                double, double, double, double, double, double
            );
        
        private:
            void clip(Robot&);
            bool step(double, Robot&);
    };
}
