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
            const double time, ds, vm, am;

            /* For A-Star */
            double* g;
            int* parent;
            bool* visited;

            /* For B-Spline */
            Eigen::Vector3d cp, cv, ca;

            /* For L-BFGS */
            const double lambda;
            const int past, mem, iterations;
            Eigen::VectorXd limit, memory, pf;
            const double eps, steps, delta, epsilon, wolfe, armijo;


        public:
            ~Planner();
            Planner(Map* world, Robot* robot, 
                    double dt, double vel, double acc,
                    double lmd, int pst, int m, int itr,
                    double e, double step, double del,
                    double ep, double wol, double arm);

        public:
            nav_msgs::Path msg(const std::string& frame, 
                               const std::vector<Eigen::Vector2d>& ctrl);
            std::vector<Eigen::Vector2d> plan(double xg, double yg);
            Eigen::Vector4d control(std::vector<Eigen::Vector2d>& path);
        
        private:
            /* A* algorithm */
            void bfs(int*, int*);
            void decode(int code, int* x, int* y) {
                *y = code / map->size.x(); *x = code - *y * map->size.x();
            }
            int encode(int x, int y) {return x + y * map->size.x();}
            double f(double g0, int x, int y, int xg, int yg) {
                return g0 + std::hypot(x - xg, y - yg);
            }
            std::vector<Eigen::Vector2d> astar(int, int, int, int);

            /* B-spline optimization */
            double optimize(Eigen::Matrix2Xd&);
            Eigen::Matrix2Xd bspline(const std::vector<Eigen::Vector2d>&);
            double cost(const Eigen::VectorXd& var, Eigen::VectorXd& grad);
            inline bool search(Eigen::VectorXd& gradient, double* step,
                               Eigen::VectorXd& x, double* value,
                               const Eigen::VectorXd& direction,
                               const Eigen::VectorXd& x0,
                               const Eigen::VectorXd& g0);
            bool convergance(const Eigen::VectorXd& x, 
                             const Eigen::VectorXd& grad)
            {
                return epsilon >= grad.cwiseAbs().maxCoeff() / std::max(
                    x.cwiseAbs().maxCoeff(), 1.
                );
            }
    };
}
