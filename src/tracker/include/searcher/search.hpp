/* @Author YueLin */

#include "searcher/map.hpp"

namespace eva_tracker
{
    class PathSearcher
    {
        private:
            double err;
            int distance;
            bool** visited;
            Point<int> size;
        
        public:
            int search;

        public:
            ~PathSearcher();
            PathSearcher(double, double, double, double, double, double);
        
        public:
            Path plan(Map&, Path&);

        private:
            Point<int> bfs(Map&, Point<int>, Point<int>);
            bool visible(Map&, Point<int>&, Point<int>&);
            Point<double> find(Map&, Point<double>, Point<double>);
    };
}
