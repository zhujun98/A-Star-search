//
// Created by jun on 8/24/17.
//

#ifndef BACHELOR_SEARCH_H
#define BACHELOR_SEARCH_H

#include <iostream>
#include <vector>
#include <limits>
#include <set>
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
struct cmpByCost {
    bool operator()(const std::pair<double, pair>& a,
                    const std::pair<double, pair>& b) const
    {
        return a.first < b.first;
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
 * @return: a vector containing a pair of the shortest distance
 *          between each grid to the source as well as the
 *          previous grid of each grid in the shortest path.
 */
std::vector<std::pair<double, pair>>
aStarSearch(const mmap::Map &map, pair src, pair dst, bool use_dijkstra=false) {
    // store the estimated smallest cost from source to destination
    // when the search reaches the corresponding grid
    std::set<std::pair<double, pair>, cmpByCost> remain;
    // Store the actual smallest cost and its previous grid
    std::vector<std::pair<double, pair>> costs(map.size());

    // initialization
    for (size_t i = 0; i < map.width(); ++i)
    {
        for (size_t j = 0; j < map.height(); ++j)
        {
            size_t idx = map.getIndex(std::make_pair(i, j));
            pair grid = std::make_pair(i, j);
            if (grid != src)
            {
                remain.insert(std::make_pair(kMaxCost, grid));
                costs[idx].first = kMaxCost;
                // uninitialized previous grid
            }
            else
            {
                remain.insert(std::make_pair(0, src));
                costs[idx].first = 0;
                costs[idx].second = src;
            }
        }
    }

    // Run until there is no grids left in the remain set.
    while (!remain.empty())
    {
        // Pick the grid in the 'remain' set with the smallest cost.
        pair pick = remain.begin()->second;
        size_t pick_idx = map.getIndex(pick);
        remain.erase(remain.begin());

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
                    double new_dist = costs[pick_idx].first + map.evalCost(pick, pts);
                    // Update smallest cost information
                    if (costs[idx].first > new_dist)
                    {
                        remain.erase(std::make_pair(costs[idx].first, pts));
                        costs[idx].first = new_dist;
                        costs[idx].second = pick;
                        // Add heuristic
                        double h = 0;
                        if (!use_dijkstra) { h = heuristicCost(pts, dst); }

                        double estimated_dist = new_dist + h;
                        remain.insert(std::make_pair(estimated_dist, pts));
                    }
                }
            }
        }
    }

    return costs;
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
    if (! map.isValid(src)) {
        throw std::invalid_argument("Source grid is not valid!");
    }

    if (! map.isValid(dst)) {
        throw std::invalid_argument("destination grid is not valid!");
    }

    if (map.isObstacle(src)) {
        throw std::invalid_argument("Source grid is unreachable!");
    }

    if (map.isObstacle(dst)) {
        throw std::invalid_argument("destination grid is unreachable!");
    }

    std::vector<std::pair<double, pair>> costs = aStarSearch(map, src, dst);

    std::stack<pair> path_tmp;
    pair last_pts = dst;
    path_tmp.push(dst);

    size_t count = 0;
    while (count++ < costs.size())
    {
        last_pts = costs[last_pts.second * map.width() + last_pts.first].second;
        path_tmp.push(last_pts);
        if (last_pts == src) { break; }
    }

    std::vector<bool> path(map.size(), false);
    while (!path_tmp.empty())
    {
        path[path_tmp.top().second * map.width() + path_tmp.top().first] = true;
        path_tmp.pop();
    }

    return std::make_pair(costs[dst.second * map.width() + dst.first].first, path);
};

} // namespace msearch

#endif //BACHELOR_SEARCH_H
