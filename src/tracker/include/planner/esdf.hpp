/* @Author YueLin */

#pragma once

#include <vector>
#include <Eigen/Eigen>

namespace eva_tracker
{
    class ESDF
    {
        private:
            int size[3];
            double*** esdf;
            Eigen::Vector3d offset;
            double resolution, maximum;
            const int X = 0, Y = 1, Z = 2;
        
        public:
            ~ESDF();

            /* Construct the ESDF */
            ESDF(double, double, double, double);   // FoV-ESDF
            ESDF(Eigen::Vector3d, double, double);  // RC-ESDF

            /* Get the index of the maximum value */
            Eigen::Vector3d argmax();

            /* Get the gradient value of ESDF */
            Eigen::Vector3d gradient(Eigen::Vector3d);

            /* Get the maximum value */
            double max() {if(maximum < 0) argmax(); return maximum;}

            /* Get the ESDF value */
            double get(Eigen::Vector3d& point) {return value(transform(point));}

        private:
            void build(bool***);
            double value(Eigen::Vector3d);
            void dt(std::vector<double>&);
            void dt(std::vector<double>&, std::vector<double>&);

            Eigen::Vector3d transform(Eigen::Vector3d& point)
            {
                return (point + offset) / resolution;
            }
            
            void interpolation(Eigen::Vector3d& p, int* coordinate, double* k)
            {
                coordinate[0] = p[X]; coordinate[1] = coordinate[0] + 1;
                coordinate[2] = p[Y]; coordinate[3] = coordinate[2] + 1;
                coordinate[4] = p[Z]; coordinate[5] = coordinate[4] + 1;
                k[0] = p[X] - coordinate[0]; k[1] = coordinate[1] - p[X];
                k[2] = p[Y] - coordinate[2]; k[3] = coordinate[3] - p[Y];
                k[4] = p[Z] - coordinate[4]; k[5] = coordinate[5] - p[Z];
            }
    };
}
