/* @Author YueLin */

#include <vector>

#include "lbfgs.hpp"

#include "utils.hpp"

namespace trajectory_prediction
{
    typedef std::vector<eva_tracker::Point<double>> Points;

    class Predictor
    {
        private:
            double gamma;
            std::vector<double> exp;
            Points observation, prediction;
            lbfgs::lbfgs_parameter_t params;

        public:
            Predictor(double, int);
            Points predict(Points&);

        private:
            static double f(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };
}
