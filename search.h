//
// Created by jun on 8/24/17.
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
 * Calculate the heuristic cost between two points.
 *
 * @param src: source point
 * @param dst: destination point
 * @return: cost
 */
double heuristicCost(pair src, pair dst)
{
    double dx = (double)src.first - (double)dst.first;
    double dy = (double)src.second - (double)dst.second;

    double cost = std::sqrt(dx*dx + dy*dy);

    return cost;
}

/**
 * Comparator
 */
struct cmpByCost
{
    bool operator()(const std::pair<double, pair>& a,
                    const std::pair<double, pair>& b) const
    {
        return a.first > b.first;
    }
};

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
                pair src,
                pair dst)
{
    // True if the point is in the shortest path
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
 * @param use_dijkstra: true for degenerating to Dijkstra's algorithm.
 * @return: a pair with the first element being the cost of the shortest
 *          path and the second element being a 1D vector similar to
 *          elevation and overrides with the point in the shortest path
 *          marked "true".
 */
std::pair<double, std::vector<bool>>
aStarSearch(const mmap::Map &map, pair src, pair dst, bool use_dijkstra=false)
{
    // The first element is the actual cost from source to the current
    // point plus the estimated cost from the current point to the
    // destination; the second element is the coordinate of the current
    // point.
    std::priority_queue<std::pair<double, pair>,
                        std::vector<std::pair<double, pair>>,
                        cmpByCost> open_set;

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
        if (pick == dst) { break; }

        // skip the old copies in the open set
        if (explored[pick_idx]) { continue; }
        explored[pick_idx] = true;

        // loop over the surrounding 8 points
        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                pair pts = std::make_pair(pick.first + i, pick.second + j);
                if (pts != pick && map.isValid(pts) && !map.isObstacle(pts))
                {
                    size_t next_idx = map.getIndex(pts);

                    double new_dist = costs[pick_idx] + map.evalCost(pick, pts);

                    // Update smallest cost information
                    if (costs[next_idx] > new_dist)
                    {
                        costs[next_idx] = new_dist;
                        came_from[next_idx] = pick;

                        // Add heuristic
                        double h = 0;
                        if (!use_dijkstra) { h = heuristicCost(pts, dst); }
                        double estimated_dist = new_dist + h;

                        open_set.push(std::make_pair(estimated_dist, pts));
                    }
                }
            }
        }
    }

    return std::make_pair(costs[map.getIndex(dst)],
                          reconstructPath(map, came_from, src, dst));
}

/**
 * Search the shortest path between two points.
 *
 * @param map: Map object
 * @param src: source point
 * @param dst: destination point
 * @return: a pair with the first element being the cost of the shortest
 *          path and the second element being a 1D vector similar to
 *          elevation and overrides with the point in the shortest path
 *          marked "true".
 */
std::pair<double, std::vector<bool>>
shortestPath(const mmap::Map &map, pair src, pair dst)
{
    if (! map.isValid(src))
    {
        throw std::invalid_argument("Source point is not valid!");
    }

    if (! map.isValid(dst))
    {
        throw std::invalid_argument("destination point is not valid!");
    }

    if (map.isObstacle(src))
    {
        throw std::invalid_argument("Source point is unreachable!");
    }

    if (map.isObstacle(dst))
    {
        throw std::invalid_argument("destination point is unreachable!");
    }

    return aStarSearch(map, src, dst, false);
};

} // namespace msearch

#endif //BACHELOR_SEARCH_H
