/* @Author YueLin */

#include <vector>

#include <Eigen/Eigen>

namespace eva_tracker
{
    class Trajectory
    {
        private:
            static const int S = 3;
            static const int D = S * 2 - 1;
        
        private:
            std::vector<double> times;
            std::vector<Eigen::MatrixXd> coefficients;
        
        public:
            Trajectory() = default;
        
        private:
            int index(double*);
            Eigen::VectorXd derivative(double, const int, const int);
            Eigen::VectorXd derivative(double time, const int n)
            {
                int idx = index(&time);
                return derivative(time, n, idx);
            }
        
        public:

            /* Get the total duration of the trajectory */
            double duration();

            /* Get the number of pieces */
            int size() const {return times.size();}

            /* Clear trajectory */
            void clear() {times.clear(); coefficients.clear();}

            /* Set durations and coefficients */
            void set(std::vector<double>& t, std::vector<Eigen::MatrixXd>& c)
            {
                times.assign(t.begin(), t.end());
                coefficients.assign(c.begin(), c.end());
            }

            /* Get the point on the trajectory at time */
            Eigen::VectorXd pos(double time) {return derivative(time, 0);}
            
            /* Get the velocity on the trajectory at time */
            Eigen::VectorXd vel(double time) {return derivative(time, 1);}

            /* Get the acceleration on the trajectory at time */
            Eigen::VectorXd acc(double time) {return derivative(time, 2);}

            /* Get the jerk on the trajectory at time */
            Eigen::VectorXd jerk(double time) {return derivative(time, 3);}
    };

    class Minco
    {
        private:
            static const int S = 3;       // S = 3 for minimize jerk
            static const int D = 4;       // 4D: x, y, z, yaw
            static const int N = S << 1;  // N = 2S
        
        private:
            int n;                     // The number of pieces
            double* a;                 // Banded system
            Eigen::MatrixXi q;         // q(i, j) = (i+S)!(j+S)! / (i!j!(i+j+1))
            int factorial[N][N];       // factorial[i][j] = i! / j!
        
        private:
            Eigen::VectorXd times[N];
            Eigen::MatrixXd b, head, tail;
        
        public:
            Minco();
            ~Minco();
        
        private:
            int index(int i, int j) {return (i - j + N) * N * n + j;}
        
        public:
            /* Initialize Minco */
            void initialize(Eigen::MatrixXd&, Eigen::MatrixXd&, int);
            
            /* Set points and times of Minco */
            void set(Eigen::MatrixXd&, Eigen::VectorXd&);

            /* Get Minco trajectory */
            void get(Trajectory*);

            /* Propagate the gradient of t to tau */
            void propogate(Eigen::VectorXd&, 
                           Eigen::VectorXd&, 
                           Eigen::VectorXd&);

            /* Propogate gradients to points and times */
            void propogate(Eigen::MatrixXd, Eigen::VectorXd&,
                           Eigen::MatrixXd&, Eigen::VectorXd&, bool);

            /* Calculate trajectory cost and gradients */
            double cost(Eigen::MatrixXd&, Eigen::VectorXd&, double);

            /* Diffeomorphic transformation of time */
            void diffeomorphism(const Eigen::VectorXd&, Eigen::VectorXd&, bool);

            /* Set the endpoint */
            void set(Eigen::VectorXd& endpoint) {tail.col(0) = endpoint;}

            /* Get matrix b */
            const Eigen::MatrixXd& coefficients() const {return b;}

            /* Calculate n! / m! */
            int div(int n, int m) const {return factorial[n][m];}

            /* Get order = 2s - 1 */
            const int order() const {return N - 1;}

            /* Get dimensionality */
            const int dim() const {return D;}

            /* Get the number of pieces */
            int pieces() const {return n;}
    };
}
