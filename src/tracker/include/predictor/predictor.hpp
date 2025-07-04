/* @Author YueLin */

#include "predictor/bezier.hpp"

namespace eva_tracker
{
    class Predictor
    {
        private:
            int m;
            int* factorial;
            Bezier* bezier;
            Eigen::VectorXd vector;
            Eigen::MatrixXd matrix;

        public:
            Predictor(Bezier*);
            ~Predictor() {delete[] factorial;};

        public:
            void predict(const Eigen::Matrix3Xd&);

        private:
            int combination(int n, int k)
            {
                if(k < 0 || k > n) return 0;
                return factorial[n] / (factorial[k] * factorial[n - k]);
            }

            double integral(int i, int j)
            {
                return std::pow(2 * bezier->duration, i + j + 1) * (
                    factorial[i] * factorial[j]
                ) / factorial[i + j + 1];
            }
    };
}
