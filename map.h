//
// Map class header.
//

#ifndef BACHELOR_MAP_H
#define BACHELOR_MAP_H

#include <iostream>
#include <vector>
#include <array>


namespace mmap
{

typedef std::pair<size_t, size_t> pair;

class Map {

private:
    const size_t width_; // width of the map
    const size_t height_; // height of the map
    // pointer to elevation, which is a vector storing the elevation
    // of each point
    const std::vector<uint8_t>* const elevation_;
    // pointer to obstacles which is a vector marking whether each
    // point is an obstacle
    const std::vector<bool>* const obstacles_;

    // check whether a point belongs to the map
    bool isValid(const pair& point) const;

    // check whether a point is an obstacle or not
    bool isObstacle(const pair& point) const;

    // calculate the difference of elevation between two points
    int elevationDiff(const pair& src, const pair& dst) const;

    // calculate the cost between two adjacent points
    //
    // Assume src and dst are adjacent and both drivable!!!
    double evalCost(const pair& src, const pair& dst) const;

    // find the maximum difference of elevation between two adjacent
    // reachable points
    void maxElevationDiff() const;

public:
    // constructor
    Map(size_t width,
        size_t height,
        const std::vector<uint8_t>& elevation,
        const std::vector<bool>& obstacles);

    // destructor
    ~Map();

    /**
     * Search the shortest path between two points.
     *
     * @param map: Map object
     * @param src: source point
     * @param dst: destination point
     * @param w1: weight for ground-truth cost (default 1.0)
     * @param w2: weight for heuristic cost (default 1.0)
     * @param verbose: true for print out message
     * @return: a pair with the first element being the cost of the shortest
     *          path and the second element being a 1D vector similar to
     *          elevation and overrides with the point in the shortest path
     *          marked "true".
     */
    std::pair<double, std::vector<bool>>
    shortestPath(const pair& src, const pair& dst,
                 double w1=1.0, double w2=1.0, bool verbose=true) const;

    // return the valid neighbor points of the source and the costs
    // between the neighbors and the source.
    //
    // use std::array as container for speed
    std::array<std::pair<pair, double>, 8> neighbors(const pair& src) const;

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
