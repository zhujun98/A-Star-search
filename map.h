//
// Map class header.
//

#ifndef BACHELOR_MAP_H
#define BACHELOR_MAP_H

#include <iostream>
#include <deque>
#include <vector>


namespace mmap
{

typedef std::pair<size_t, size_t> pair;

class Map {

private:
    const size_t width_; // width of the map
    const size_t height_; // height of the map
    const std::vector<uint8_t>* const elevation_; // pointer to elevation
    const std::vector<uint8_t>* const overrides_; // pointer to overrides
    const uint8_t of_river_marsh_; // mark for river marsh
    const uint8_t of_water_basin_; // mark for water basin

    // check whether a point belongs to the map
    bool isValid(const pair& point) const;

    // check whether a point is an obstacle (water, marsh)
    bool isObstacle(const pair& point) const;

    // calculate the different of elevation between two points
    int elevationDiff(const pair& src, const pair& dst) const;

    // find the maximum difference of elevation between two adjacent
    // reachable points
    void maxElevationDiff() const;

public:
    // constructor
    Map(size_t width,
        size_t height,
        const std::vector<uint8_t>& elevation,
        const std::vector<uint8_t>& overrides,
        uint8_t of_river_marsh,
        uint8_t of_water_basin);

    // destructor
    ~Map();

    /**
     * Search the shortest path between two points.
     *
     * @param map: Map object
     * @param src: source point
     * @param dst: destination point
     * @param fast_search: true for fast search but the result path maybe
     *                     longer than the actual shortest path; false for
     *                     guaranteed shortest path if it exists.
     * @param verbose: true for print out message
     * @return: a pair with the first element being the cost of the shortest
     *          path and the second element being a 1D vector similar to
     *          elevation and overrides with the point in the shortest path
     *          marked "true".
     */
    std::pair<double, std::vector<bool>>
    shortestPath(const pair& src, const pair& dst, bool fast_search=false,
                 bool verbose=true) const;

    // calculate the cost between two adjacent points
    double evalCost(const pair& src, const pair& dst) const;

    // return the valid neighbor points of the source
    std::deque<pair> neighbors(const pair& src) const;

    // get width of the map
    size_t width() const;

    // get height of the map
    size_t height() const;

    // get number of points
    size_t size() const;

    // get the index when the 2d array is flatten to 1D
    size_t getIndex(const pair& point) const;
};

}  // namespace mmap

#endif //BACHELOR_MAP_H
