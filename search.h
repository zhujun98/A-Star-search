//
// A* search algorithm.
//

#ifndef BACHELOR_SEARCH_H
#define BACHELOR_SEARCH_H

#include <iostream>
#include <vector>
#include <queue>
#include <stack>

#include "map.h"

namespace msearch {

const double kMaxCost = 1.0e20;

typedef std::pair<size_t, size_t> pair;


/**
 * Calculate the Octile distance between two points.
 *
 * @param src: source point
 * @param dst: destination point
 * @return: cost
 */
inline double heuristicOctile(const pair& src, const pair& dst)
{
    double dx = std::abs((double)src.first - (double)dst.first);
    double dy = std::abs((double)src.second - (double)dst.second);

    return ((dx > dy) ? dx + 0.414213562*dy : dy + 0.414213562*dx);
}

/**
 * Comparator
 */
//struct cmpByCost
//{
//    bool operator()(const std::pair<double, pair>& a,
//                    const std::pair<double, pair>& b) const
//    {
//        return a.first > b.first;
//    }
//};

/**
 * Reconstruct the shortest path from the search result
 *
 * @param map: Map object
 * @param came_from: a vector that stores the previous point of each
 *                   point in the shortest path
 * @param src: source point
 * @param dst: destination point
 * @return: a 1D vector similar to elevation and overrides with the
 *          point in the shortest path marked "true".
 */
std::vector<bool>
reconstructPath(const mmap::Map &map,
                const std::vector<pair>& came_from,
                const pair& src,
                const pair& dst)
{
    // The space complexity is high but it facilitate the later map marking.
    std::vector<bool> path(came_from.size(), false);

    pair prev_point = dst;
    size_t count = 0;
    while (++count < came_from.size())
    {
        size_t idx = map.getIndex(prev_point);
        path[idx] = true;
        prev_point = came_from[idx];
        if (prev_point == src)
        {
            path[map.getIndex(src)] = true;
            break;
        }
    }

    return path;
}

/**
 * Find the shortest path using the A* algorithm implemented utilizing
 * priority queue.
 *
 * @param map: Map object
 * @param src: source point
 * @param dst: destination point
 * @param w1: weight for ground-truth cost (default 1.0)
 * @param w2: weight for heuristic cost (default 1.0)
 *            Note: set w1 = 1.0, w2 = 0.0 to use the Dijkstra's algorithm
 *                  set w1 = 0.0, w2 = 1.0 to use the pure heuristic search
 * @return: a pair with the first element being the cost of the shortest
 *          path and the second element being a 1D vector similar to
 *          elevation and overrides with the point in the shortest path
 *          marked "true". Throw std::invalid_argument if path does not
 *          exist.
 */
std::pair<double, std::vector<bool>>
aStarSearch(const mmap::Map &map, const pair& src, const pair& dst,
            double w1=1.0, double w2=1.0)
{
    // The first element is the actual cost from source to the current
    // point plus the estimated cost from the current point to the
    // destination; the second element is the coordinate of the current
    // point.

    auto cmp = [](const std::pair<double, pair>& a,
                  const std::pair<double, pair>& b)
    {
      return (a.first > b.first);
    };
    std::priority_queue<std::pair<double, pair>,
                        std::vector<std::pair<double, pair>>,
                        decltype(cmp)> open_set(cmp);

//    std::priority_queue<std::pair<double, pair>,
//        std::vector<std::pair<double, pair>>,
//        cmpByCost> open_set;

    // actual minimum cost to the source so far
    std::vector<double> costs(map.size(), kMaxCost);
    // the previous point that allows to achieve the minimum cost so far
    std::vector<pair> came_from(map.size());
    // mark the explored point
    std::vector<bool> explored(map.size(), false);

    // initialization
    size_t src_idx = map.getIndex(src);
    costs[src_idx] = 0;
    came_from[src_idx] = src;

    // Run until there is no points left in the open set.
    open_set.push(std::make_pair(0, src));
    while (!open_set.empty())
    {
        // Pick the point in the open set with the smallest cost.
        pair pick = open_set.top().second;
        size_t pick_idx = map.getIndex(pick);
        open_set.pop();

        // stop search when reach the destination
        if (pick == dst)
        {
            return std::make_pair(costs[map.getIndex(dst)],
                                  reconstructPath(map, came_from, src, dst));
        }

        // skip the old copies in the open set
        if (explored[pick_idx]) { continue; }
        explored[pick_idx] = true;

        // loop over neighbor points
        for (const auto &v : map.neighbors(pick))
        {
            size_t next_idx = map.getIndex(v.first);
            double new_dist = costs[pick_idx] + v.second;
            // Update smallest cost information
            if (costs[next_idx] > new_dist)
            {
                costs[next_idx] = new_dist;
                came_from[next_idx] = pick;
                // Add heuristic
                double h = heuristicOctile(v.first, dst)*w2;
                open_set.push(std::make_pair(new_dist*w1 + h, v.first));
            }
        }
    }

    throw std::invalid_argument(
        "Invalid argument: source and destination are not connected!");
}

} // namespace msearch

#endif //BACHELOR_SEARCH_H
