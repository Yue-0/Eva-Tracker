/* @Author: YueLin */

#include <utility>

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace simulator
{
    class Planner
    {
        private:
            Map* map;
            Robot* car;
            double time, ds, vm, am;

        public:
            Planner(Map*, Robot*, double, double, double);

        public:
            R3xSO2 control(std::vector<std::pair<double, double>>&);
            std::vector<std::pair<double, double>> plan(double, double);
            nav_msgs::Path msg(
                std::string&, std::vector<std::pair<double, double>>&
            );
        
        private:
            void bfs(int*, int*);

            /* A* algorithm */
            void decode(int code, int* x, int* y, const int cols){
                *y = code / cols; *x = code - *y * cols;
            }
            int encode(int x, int y, const int cols) {return x + y * cols;}
            double f(double g, int x, int y, int xg, int yg) {
                return g + std::hypot(x - xg, y - yg);
            }
            std::vector<std::pair<double, double>> astar(int, int, int, int);

            /* B-spline optimization */
            std::vector<std::pair<double, double>> bspline(
                const std::vector<std::pair<double, double>>&
            );
            double optimize(std::vector<std::pair<double, double>>&);
    };
}
