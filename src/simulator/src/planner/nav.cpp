/* @Author: YueLin */

#include <queue>
#include <utility>

#include "planner/nav.hpp"

namespace nav
{
    HybirdAStar::HybirdAStar(sim::Robot* robot,
                             double dt, double err,
                             double max_acc, double safe,
                             double max_vel, double max_angle,
                             double penalty_back, double penalty_trun, int step)
    {
        limit[VEL] = max_vel;
        limit[ACC] = max_acc;
        ds = safe; car = robot;
        time = dt; error = err;
        limit[RAD] = max_angle * time * PI / 180;
        interval[VEL] = limit[VEL] * time / step;
        interval[RAD] = limit[RAD] / step;
        penalty[VEL] = penalty_back;
        penalty[RAD] = penalty_trun;
    }

    std::vector<Node> HybirdAStar::plan(sim::Map& map,
                                        std::vector<std::vector<double>> h)
    {
        /* Initialize */
        nodes.clear();
        const double r = 1 / map.resolution;
        std::priority_queue<std::pair<double, int>> open;
        const int X = map.size[sim::X], Y = map.size[sim::Y];

        /* Construction start point */
        Node start;
        start.v = car->vel.x;
        start.w = car->vel.yaw; 
        start.g = start.parent = start.id = 0;
        double sin0 = std::sin(car->pose.yaw);
        double cos0 = std::cos(car->pose.yaw);
        start.yaw = car->pose.yaw + start.w * time;
        double sin = std::sin(start.yaw), cos = std::cos(start.yaw);
        if(std::fabs(car->vel.yaw) <= O)
            start.x = car->pose.x + start.v * time * cos,
            start.y = car->pose.y + start.v * time * sin;
        else
            start.x = car->pose.x + start.v * (sin - sin0) / start.w,
            start.y = car->pose.y + start.v * (cos0 - cos) / start.w;
        double hm = std::hypot(map.size0[sim::X], map.size0[sim::Y]);
        int x0 = std::max(std::min(std::round(r * start.x), X - 1.), 0.);
        int y0 = std::max(std::min(std::round(r * start.y), Y - 1.), 0.);
        start.h = std::min(hm, h[x0][y0]);

        /* A* algorithm */
        nodes.push_back(start);
        double timeout = time + TIME();
        open.push(std::make_pair(-f(start), 0));
        while(!open.empty())
        {
            Node node = nodes[open.top().second]; open.pop();

            /* If found a path or timeout */
            if(node.h <= error || timeout <= TIME())
            {
                std::vector<Node> trajectory;
                trajectory.push_back(node);
                while(node.parent != start.id)
                {
                    node = nodes[node.parent];
                    trajectory.push_back(node);
                }
                trajectory.push_back(start);
                return trajectory;  // Please note that this is a reverse path
            }

            /* Expand node */
            for(Node next: expand(map, node))
            {
                /* Calculate cost */
                double cost = std::hypot(node.x - next.x, node.y - next.y);
                
                /* Build node information */
                next.parent = node.id;
                next.id = nodes.size();
                next.g = node.g + cost;

                /* Back penalty and turn penalty */
                if(next.v < -O)
                    next.g += cost * penalty[VEL] * limit[VEL];
                if(std::fabs(next.w) > O)
                    next.g += cost * penalty[RAD] * std::fabs(next.w);
                
                /* Calculate heuristic value */
                x0 = std::max(std::min(std::round(r * next.x), X - 1.), 0.);
                y0 = std::max(std::min(std::round(r * next.y), Y - 1.), 0.);
                next.h = std::min(hm, h[x0][y0]);

                /* Add node to open list */
                nodes.push_back(next);
                open.push(std::make_pair(-f(next), next.id));
            }
        }

        /* No path found */
        return std::vector<Node>();
    }

    std::vector<std::vector<double>> HybirdAStar::dijkstra(sim::Map& map,
                                                           float sx, float sy)
    {
        /* Initialize */
        double dist, r = 1 / map.resolution;
        const int X = map.size[sim::X], Y = map.size[sim::Y];
        std::vector<std::vector<bool>> visited(
            X, std::vector<bool>(Y, false)
        );
        std::vector<std::vector<double>> distance(
            X, std::vector<double>(Y, INF)
        );
        const int size = std::round(std::min(car->width, car->height) * r / 2); 
        int x0 = std::round(sx * r), y0 = std::round(sy * r);
        int z = std::round(car->height * r);

        /* Dijkstra's algorithm */
        std::priority_queue<std::pair<double, int>> queue;
        queue.push(std::make_pair(distance[x0][y0] = 0, y0 + x0 * X));
        while(!queue.empty())
        {
            /* Get the nearest node */
            int index = queue.top().second; queue.pop();
            x0 = index / X; y0 = index % Y;
            
            /* Expand neighbor nodes */
            for(int x = std::max(size, x0 - 1); x <= x0 + 1; x++)
            {
                if(x >= X - size) break;
                for(int y = std::max(size, y0 - 1); y <= y0 + 1; y++)
                {
                    if(y >= Y - size) break;
                    if(visited[x][y]) continue;
                    bool collision = false;
                    for(int sgn = -size; sgn <= size && !collision; sgn += size)
                        collision = (
                           map.exp[x + sgn][y][0] ||
                           map.exp[x + sgn][y][z] ||
                           map.exp[x][y + sgn][0] ||
                           map.exp[x][y + sgn][z]
                        );
                    if(!collision)
                    {
                        dist = distance[x0][y0] + std::hypot 
                               (x - x0, y - y0) * map.resolution;
                        if(dist < distance[x][y])
                        {
                            distance[x][y] = dist;
                            queue.push(std::make_pair(-dist, y + x * X));
                        }
                    }
                } 
            }
            visited[x0][y0] = true;
        }
        return distance;
    }

    nav_msgs::Path HybirdAStar::msg(std::vector<Node>& path, std::string& frame)
    {
        /* Convert std::vector<Node> to nav_msgs::Path */
        nav_msgs::Path message;
        message.header.frame_id = frame;
        for(Node node: path)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = node.x;
            pose.pose.position.y = node.y;
            pose.header.frame_id = frame;
            message.poses.push_back(pose);
        }
        std::reverse(message.poses.begin(), message.poses.end());
        return message;
    }

    sim::SO3 HybirdAStar::control(std::vector<Node>& trajectory)
    {
        sim::SO3 ctrl = {0.0, 0.0, 0.0, 0.0};
        if(trajectory.empty()) return ctrl;
        Node node = trajectory.back();
        trajectory.pop_back();
        ctrl.yaw = node.w;
        ctrl.x = node.v;
        return ctrl;
    }

    double HybirdAStar::f(Node& node)
    {
        return node.g + node.h + std::max(
            node.v * node.v / (limit[ACC] * 2) - node.h, 0.
        );
    }

    std::vector<Node> HybirdAStar::expand(sim::Map& map, Node& node)
    {
        std::vector<Node> neighbors;
        double r = 1 / map.resolution;
        double v0 = std::trunc(std::max(
            node.v - time * limit[ACC], -limit[VEL]
        ) / interval[VEL]) * interval[VEL];
        int z0 = std::round(car->height * r);
        double sin0 = std::sin(node.yaw), cos0 = std::cos(node.yaw);
        double vm = std::min(node.v + time * limit[ACC], limit[VEL]);
        for(double rad = -limit[RAD]; rad <= limit[RAD]; rad += interval[RAD])
        {
            double x, y;
            double theta = node.yaw + rad;
            if(theta > PI) theta -= 2 * PI;
            if(theta <= -PI) theta += 2 * PI;
            double sin = std::sin(theta), cos = std::cos(theta);
            double ws = car->width * sin / 2, wc = car->width * cos / 2;
            double ls = car->length * sin / 2, lc = car->length * cos / 2;
            for(double vel = v0; vel <= vm; vel += interval[VEL])
            {
                if(std::fabs(vel) <= O)
                    continue;

                /* Expand */
                if(std::fabs(rad) <= O)
                    x = node.x + vel * time * cos,
                    y = node.y + vel * time * sin;
                else
                    x = node.x + vel * time * (sin - sin0) / rad,
                    y = node.y + vel * time * (cos0 - cos) / rad;
                
                /* Detect collision */
                bool collision = false;

                double keypoints[9][2] = {
                    x * r, y * r,
                    (x - lc - ws) * r, (y + wc - ls) * r,
                    (x + lc - ws) * r, (y + wc + ls) * r,
                    (x + lc + ws) * r, (y - wc + ls) * r,
                    (x - lc + ws) * r, (y - wc - ls) * r
                };
                for(int p = 0; p < 4; p++)
                    for(int z = 0; z < 2; z++)
                    {
                        keypoints[p + 5][z] = (
                            keypoints[p + 1][z] + keypoints[p % 4 + 2][z]
                        ) / 2;
                    }
                for(int p = 0; p < 9; p++)
                {
                    int px = std::round(keypoints[p][sim::X]);
                    int py = std::round(keypoints[p][sim::Y]);
                    if((collision = std::min(px, py) < 0 ||
                        px >= map.size[sim::X] || py >= map.size[sim::Y] ||
                        map.exp[px][py][z0] || map.exp[px][py][0])) break;
                }
                if(!collision)
                {
                    float n = std::max(
                        map.resolution / std::max(x - node.x, y - node.y), 0.1
                    );
                    for(float t = n; t < 1 - O; t += n)
                    {
                        int px = std::round(((1 - t) * node.x + t * x) * r);
                        int py = std::round(((1 - t) * node.y + t * y) * r);
                        if((collision = map.exp[px][py][0])) break;
                    }
                }

                /* Add neighbor */
                if(!collision)
                {
                    Node neighbor;
                    neighbor.x = x;
                    neighbor.y = y;
                    neighbor.v = vel;
                    neighbor.yaw = theta;
                    neighbor.w = rad / time;
                    neighbors.push_back(neighbor);
                }
            }
        }
        return neighbors;
    }
}
