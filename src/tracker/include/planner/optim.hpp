/* @Author YueLin */

#include "lbfgs"

#include "planner/map.hpp"
#include "planner/esdf.hpp"
#include "planner/minco.hpp"
#include "predictor/bezier.hpp"

namespace eva_tracker
{
    class Optimizer
    {
        private:
            int iterations;
            const double PI = std::acos(-1);

        public:
            int kappa;
            double duration;
            Eigen::VectorXd costs;

            /* Objects */
            Minco* minco;
            Bezier* bezier;
            ESDF *robot, *fov;
            pcl::PointCloud<pcl::PointXYZ>* cloud;

            /* Hyperparameters */
            double vh2, vv2, va2, ah2, av2, aa2;
            double lambda_p, lambda_o, lambda_d, lambda_a, gamma, weight;

            /* Vectors */
            Eigen::MatrixXd points, dc, dp;
            Eigen::VectorXd tau, times, dt, dv, endpoint;
        
        private:
            lbfgs::lbfgs_parameter_t params;
        
        public:
            Optimizer(int, int,
                      Minco*, Bezier*, ESDF*, ESDF*,
                      double, double, double, int,
                      double, double, double, double, double, double,
                      double, double, double, double, double, double);
        
        public:
            double optimize(Trajectory*);
            bool setup(Eigen::MatrixXd&,
                       Eigen::MatrixXd*,
                       pcl::PointCloud<pcl::PointXYZ>*);

        private:
            inline void integral();
            inline void regularization();
            inline void forward(Eigen::VectorXd&,
                                Eigen::VectorXd&,
                                Eigen::MatrixXd&);
            inline void backward(const Eigen::VectorXd&);

            static double f(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };
}
