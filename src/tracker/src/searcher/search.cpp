/* @Author YueLin */

#include <queue>

#include "searcher/search.hpp"

namespace eva_tracker
{
    PathSearcher::~PathSearcher()
    {
        for(int x = 0; x < size.x; x++)
        {
            for(int y = 0; y < size.y; y++)
                delete[] visited[x][y];
            delete[] visited[x];
        }
        delete[] visited;
    }

    PathSearcher::PathSearcher(double t,
                               double dist, double length, double width,
                               double height, double error, double resolution)
    {
        /* Initialize */
        timeout = t;
        err = 1 / resolution;
        distance = std::round(dist * err);
        size.z = std::round(height * err);
        size.x = std::round(length * err);
        size.y = std::round(width * err);
        size.yaw = 0;
        err = error;

        /* Allocate memory */
        visited = new bool**[size.x];
        for(int x = 0; x < size.x; x++)
        {
            visited[x] = new bool*[size.y];
            for(int y = 0; y < size.y; y++)
                visited[x][y] = new bool[size.z];
        }
    }

    Path PathSearcher::plan(Map& map, Path& prediction)
    {
        /* Initialize */
        Path path;
        const int n = prediction.size();
        Point<double> end, start = size * (map.resolution / 2);

        /* Find the best visible path */
        path.push_back(end = start);
        if(n <= 0) return path;
        double time = TIME();
        for(int p = 1; p < n; p++)
        {
            if((end = find(map, start, prediction[p])) == start)
                break;
            path.push_back(start = end);
            if(TIME() - time > timeout)
                break;
        }
        time = TIME() - time;
        ROS_INFO("[Path] Duration: %fs.", time);
        return path;
    }

    Point<int> PathSearcher::bfs(Map& map, Point<int> start, Point<int> target)
    {
        /* Determine whether the search is need */
        if(visible(map, start, target))
        {
            VISIBLE:
                start.yaw = std::atan2(target.y - start.y, target.x - start.x);
                return start;
        }

        /* Initialize */
        std::queue<Point<int>> queue;
        for(int x = 0; x < size.x; x++)
            for(int y = 0; y < size.y; y++)
                for(int z = 0; z < size.z; z++)
                    visited[x][y][z] = false;
        start.x = std::max(std::min(start.x, size.x - 1), 0);
        start.y = std::max(std::min(start.y, size.y - 1), 0);
        start.z = std::max(std::min(start.z, size.z - 1), 0);
        visited[start.x][start.y][start.z] = true;

        /* BFS algorithm */
        queue.push(start);
        while(!queue.empty())
        {
            /* Dequeue a node */
            start = queue.front(); queue.pop();

            /* If found a visible point */
            if(visible(map, start, target))
                break;
            
            /* Expand node */
            for(Point<int> node: map.neighbors(start, 1))
                if(!visited[node.x][node.y][node.z])
                {
                    queue.push(node);
                    visited[node.x][node.y][node.z] = true;
                }
        }

        goto VISIBLE;
    }

    bool PathSearcher::visible(Map& map, Point<int>& node, Point<int>& target)
    {
        if(std::max(std::fabs(node.distance(target.x, target.y) - distance),
                    std::fabs(node.z - target.z)) * map.resolution <= err)
        {
            int x, y, z,
            dx = node.x - target.x,
            dy = node.y - target.y,
            dz = node.z - target.z;
            float step = 1. / std::max(std::max(
                std::abs(dx), std::abs(dy)
            ), std::abs(dz));
            for(float t = step; t <= 1; t += step)
            {
                x = std::round(dx * t + target.x);
                y = std::round(dy * t + target.y);
                z = std::round(dz * t + target.z);
                if(map.get(x, y, z)) return false;
            }
            return true;
        }
        return false;
    }

    Point<double> PathSearcher::find(Map& map,
                                     Point<double> start,
                                     Point<double> target)
    {
        Point<double> best = target - (distance * map.resolution / std::max(
            target.distance(start.x, start.y), O
        )) * (target - start);
        double r = 1 / map.resolution; best.z = target.z;
        return bfs(map, best.integer(r), target.integer(r)) * map.resolution;
    }
}
