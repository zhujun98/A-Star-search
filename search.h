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

const double kMaxCost = 1.0e10;

typedef std::pair<size_t, size_t> pair;


/**
 * Calculate the heuristic cost between two grids.
 *
 * @param src: source grid
 * @param dst: destination grid
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
 * Comparator for significantly reducing the insertion and deletion
 * in std::set.
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
 * Find the shortest path using the A* algorithm.
 *
 * A self-balanced RB-tree (std::set) is used to store the unfinished
 * grids and its current shortest distance to the source.
 * Time complexity < O(ElogV).
 *
 * @param map: Map object
 * @param src: source grid
 * @param dst: destination grid
 * @param use_dijkstra: true for degenerating to Dijkstra's algorithm.
 * @return: a pair containing a vector of the shortest distance
 *          between each grid to the source and a vector of the
 *          previous grid of each grid in the shortest path.
 */
std::pair<std::vector<double>, std::vector<pair>>
aStarSearch(const mmap::Map &map, pair src, pair dst, bool use_dijkstra=false)
{
    // store the estimated smallest cost from source to destination
    // when the search reaches the corresponding grid
    std::priority_queue<std::pair<double, pair>,
                        std::vector<std::pair<double, pair>>,
                        cmpByCost> remain;

    // minimum cost to the source so far
    std::vector<double> costs(map.size(), kMaxCost);
    // the previous grid that allows to achieve the minimum cost so far
    std::vector<pair> came_from(map.size());
    // initialization
    size_t src_idx = map.getIndex(src);
    costs[src_idx] = 0;
    came_from[src_idx] = src;

    // Run until there is no grids left in the remain set.
    remain.push(std::make_pair(0, src));
    while (!remain.empty())
    {
        // Pick the grid in the 'remain' set with the smallest cost.
        pair pick = remain.top().second;
        size_t pick_idx = map.getIndex(pick);
        remain.pop();

        // stop search when reach the destination
        if (pick == dst) { break; }

        // loop over the surrounding 8 grids
        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                pair pts = std::make_pair(pick.first + i, pick.second + j);
                if (pts != pick && map.isValid(pts) && !map.isObstacle(pts))
                {
                    size_t idx = map.getIndex(pts);

                    double new_dist = costs[pick_idx] + map.evalCost(pick, pts);

                    // Update smallest cost information
                    if (costs[idx] > new_dist)
                    {
                        costs[idx] = new_dist;
                        came_from[idx] = pick;

                        // Add heuristic
                        double h = 0;
                        if (!use_dijkstra) { h = heuristicCost(pts, dst); }
                        double estimated_dist = new_dist + h;

                        remain.push(std::make_pair(estimated_dist, pts));
                    }
                }
            }
        }
    }

    return std::make_pair(costs, came_from);
}

/**
 * Search the shortest path between two grids
 *
 * @param map: Map object
 * @param src: source grid
 * @param dst: destination grid
 * @return: a pair with the first element being the cost of the shortest
 *          path and the second element being a 1D vector similar to
 *          elevation and overrides with the grid in the shortest path
 *          marked "true".
 */
std::pair<double, std::vector<bool>>
shortestPath(const mmap::Map &map, pair src, pair dst)
{
    if (! map.isValid(src))
    {
        throw std::invalid_argument("Source grid is not valid!");
    }

    if (! map.isValid(dst))
    {
        throw std::invalid_argument("destination grid is not valid!");
    }

    if (map.isObstacle(src))
    {
        throw std::invalid_argument("Source grid is unreachable!");
    }

    if (map.isObstacle(dst))
    {
        throw std::invalid_argument("destination grid is unreachable!");
    }

    std::pair<std::vector<double>, std::vector<pair>>
        result = aStarSearch(map, src, dst);

    // True if the grid is in the shortest path
    // The space complexity is high but it facilitate the later map marking.
    std::vector<bool> path(result.first.size(), false);

    pair prev_grid = dst;
    size_t count = 0;
    while (++count < result.first.size())
    {
        size_t idx = map.getIndex(prev_grid);
        path[idx] = true;
        prev_grid = result.second[idx];
        if (prev_grid == src)
        {
            path[map.getIndex(src)] = true;
            break;
        }
    }

    return std::make_pair(result.first[map.getIndex(dst)], path);
};

} // namespace msearch

#endif //BACHELOR_SEARCH_H
