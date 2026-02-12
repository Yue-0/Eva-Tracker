/* @Author YueLin */

#pragma once

#include <limits>
#include <vector>

#include "Eigen/Eigen"

namespace eva_tracker
{
    class ESDF
    {
        private:
            double*** esdf;
            Eigen::Vector3i size;
            Eigen::Vector3d offset;
            double resolution, maximum;
            const double inf = std::numeric_limits<double>::infinity();
        
        public:
            ~ESDF();

            /* Construct the FoV-ESDF */
            ESDF(double alpha, double beta, double distance, double r);

            /* Construct the RC-ESDF */
            ESDF(const Eigen::Vector3d& robot, double r, double expansion);

            /* Get the ESDF value and gradient */
            double value(Eigen::Vector3d point, Eigen::Vector3d* grad) const;

            /* Get the index of the maximum value */
            Eigen::Vector3d argmax();

            /* Get the maximum value */
            double max()
            {
                if(maximum < 0)
                    argmax();
                return maximum;
            }

        private:
            void build(bool***);
            void transform(std::vector<double>&, std::vector<double>&) const;
    };
}
