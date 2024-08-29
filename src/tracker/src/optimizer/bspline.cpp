/* @Author: YueLin */

#include "optimizer/bspline.hpp"

namespace eva_tracker
{
    Path BSpline::trajectory()
    {
        Path path;
        const double k = 1 / 6.;
        for(int p = 0; p < n - 1; p++)
        {
            Point<double> point = k * (
                4 * control[p + 1] + 
                control[p] + control[p + 2]
            );
            point.yaw = k * (
                4 * control[p + 1].yaw + 
                control[p].yaw + control[p + 2].yaw
            );
            path.push_back(point);
        }
        return path;
    }

    Point<double> BSpline::operator[](double time)
    {
        double t;
        Path points;
        time = std::min(std::max(time, knots[degree]), knots[n + 1]);
        const int k = std::ceil(time / dt) + degree - 1;
        for(int left = 0; left <= degree; left++)
            points.push_back(control[k - degree + left]);
        for(int left = 1; left <= degree; left++)
            for(int right = degree; right >= left; right--)
            {
                t = (time - knots[right + k - degree]) / (
                    knots[right + k - left + 1] - knots[right + k - degree]
                );
                points[right] = t * points[right] + (1 - t) * points[right - 1];
            }
        return points[degree];
    }

    BSpline BSpline::derivative(const unsigned int order)
    {
        /* Control points of the derivative */
        Path points(n);
        for(int p = 0; p < n; p++)
            points[p] = (
                degree / (knots[p + degree + 1] - knots[p + 1])
            ) * (control[p + 1] - control[p]);
        
        /* The derivative is also a B-spline */
        BSpline spline(degree - 1);
        spline.update(points, dt);
        return order > 1? spline.derivative(order - 1): spline;
    }

    void BSpline::update(Path& points, const double t)
    {
        dt = t; control = points; update();
    }

    void BSpline::create(Path& path,
                         Velocity& vel,
                         Acceleration& acc,
                         const double t)
    {
        /* Initialize constants */
        dt = t;
        const int P = 0, V = 1, A = 2;
        const int numbers = path.size();
        const double k[3] = {1 / 6., 0.5 / dt, 1 / (dt * dt)};
        
        /* Initialize equations */
        using namespace Eigen;
        MatrixXd m = MatrixXd::Zero(numbers + 4, numbers + 2);
        Vector3d p(3), v(3), a(3); p << 1, 4, 1; v << -1, 0, 1; a << 1, -2, 1;
        VectorXd w(numbers + 4), x(numbers + 4), y(numbers + 4), z(numbers + 4);
        for(int point = 0; point < numbers; point++)
        {
            x[point] = path[point].x;
            y[point] = path[point].y;
            z[point] = path[point].z;
            w[point] = path[point].yaw;
            m.block(point, point, 1, 3) = k[P] * p.transpose();
        }
        for(int point = 1; point < numbers; point++)
        {
            while(w[point] - w[point - 1] > PI)
                w[point] -= 2 * PI;
            while(w[point] - w[point - 1] <= -PI)
                w[point] += 2 * PI;
        }
        x[numbers + 2] = acc.x; x[numbers] = vel.x;
        y[numbers + 2] = acc.y; y[numbers] = vel.y;
        z[numbers + 2] = acc.z; z[numbers] = vel.z;
        w[numbers + 2] = acc.yaw; w[numbers] = vel.yaw;
        m.block(numbers, 0, 1, 3) = k[V] * v.transpose();
        m.block(numbers + 2, 0, 1, 3) = k[A] * a.transpose();
        m.block(numbers + 1, numbers - 1, 1, 3) = k[V] * v.transpose();
        m.block(numbers + 3, numbers - 1, 1, 3) = k[A] * a.transpose();
        w[numbers + 1] = x[numbers + 1] = y[numbers + 1] = z[numbers + 1] = 0;
        w[numbers + 3] = x[numbers + 3] = y[numbers + 3] = z[numbers + 3] = 0;

        /* Solve control points */
        control.resize(numbers + 2);
        ColPivHouseholderQR<MatrixXd> qr = m.colPivHouseholderQr(); VectorXd
        cx = qr.solve(x), cy = qr.solve(y), cz = qr.solve(z), yaw = qr.solve(w);
        for(int i = 0; i < numbers + 2; i++)
        {
            control[i].x = cx[i];
            control[i].y = cy[i];
            control[i].z = cz[i];
            control[i].yaw = yaw[i];
        }

        /* Generate knots */
        update();
    }

    Eigen::VectorXd BSpline::controls()
    {
        Eigen::VectorXd vector((n + 1) << 2);
        for(int p = 0; p <= n; p++)
        {
            int index = p << 2;
            vector[index] = control[p].yaw;
            vector[index + 1] = control[p].x;
            vector[index + 2] = control[p].y;
            vector[index + 3] = control[p].z;
        }
        return vector;
    }

    void BSpline::update()
    {
        n = control.size() - 1;
        int m = n + degree + 1;
        knots = std::vector<double>(m + 1, 0);
        for(int p = 0; p <= m; p++) knots[p] = (p - degree) * dt;
    }
}
