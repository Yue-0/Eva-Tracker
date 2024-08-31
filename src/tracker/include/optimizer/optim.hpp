/* @Author YueLin */

#include "lbfgs.hpp"

#include "searcher/map.hpp"
#include "optimizer/esdf.hpp"
#include "optimizer/bspline.hpp"

namespace eva_tracker
{
    class Optimizer
    {
        public:
            double time;
            Path predict;
            int iterations;
            BSpline spline;
            ESDF *robot, *fov;
            PointCloud cloud, depth;
            double lambda[6], limit2[2][2], costs[6];
        
        private:
            lbfgs::lbfgs_parameter_t params;
        
        public:
            Optimizer(ESDF*, ESDF*, int, double,
                      double, double, double, double,
                      double, double, double, double, double, double);
        
        public:
            BSpline optimize(BSpline&, Path*, PointCloud*, PointCloud*);
        
        private:
            static double f(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };
}
