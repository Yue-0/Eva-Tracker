/* @Author YueLin */

#include "planner/map.hpp"
#include "planner/esdf.hpp"
#include "planner/minco.hpp"
#include "predictor/bezier.hpp"

namespace eva_tracker
{
    class Optimizer
    {
        private:
            const double PI = std::acos(-1);
            enum Costs {Jo = 0, Jv, Ja, Je, Jt, Jp, Nc};

            /* Objects */
            Bezier* bezier;
            Minco<4>* minco;
            ESDF *robot, *fov;
            pcl::PointCloud<pcl::PointXYZ>* cloud;

            /* Hyperparameters for optimization */
            const int kappa;
            const double duration;
            const double lambda[Nc];
            const Eigen::Vector3d v2, a2;

            /* Hyperparameters for L-BFGS */
            const int mem, past, iterations;
            const double epsilon, delta, eps, armijo, wolfe, steps;

            /* Variables */
            int iters;
            Eigen::MatrixX4d dc;
            Eigen::Matrix4Xd pts, dp;
            Eigen::Vector4d endpoint;
            Eigen::VectorXd costs, tau, times, dt, dv;
        
        public:
            Optimizer(int k, Minco<4>* m, Bezier* b, 
                      ESDF* rc, ESDF* cam, double tau, 
                      double vh, double vv, double va, 
                      double ah, double av, double aa,
                      double lp, double lo, double lv, 
                      double la, double gm, double lw, 
                      double ep, double del, double e,
                      double a, double w, double step,
                      int memory, int pst, int iteration);
        
        public:
            double optimize(Trajectory*);
            bool setup(const Eigen::Matrix4Xd& path, Eigen::Matrix4Xd* se,
                       pcl::PointCloud<pcl::PointXYZ>* world);

        private:
            double lbfgs(Eigen::VectorXd& x);
            void forward(Eigen::VectorXd& data,
                         const Eigen::VectorXd& vector,
                         const Eigen::Matrix4Xd& matrix);
            void backward(const Eigen::VectorXd& vector);
            double f(const Eigen::VectorXd& var, Eigen::VectorXd& gradients);
    };
}
