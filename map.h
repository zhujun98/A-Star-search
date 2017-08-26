//
// Created by jun on 8/24/17.
//

#ifndef BACHELOR_MAP_H
#define BACHELOR_MAP_H

#include <iostream>
#include <string>


namespace mmap
{

typedef std::pair<size_t, size_t> pair;

class Map {

private:
    size_t width_; // width of the map
    size_t height_; // height of the map
    std::vector<uint8_t>* elevation_; // pointer to elevation
    std::vector<uint8_t>* overrides_; // pointer to overrides

public:
    // constructor
    Map(size_t width,
        size_t height,
        std::vector<uint8_t>& elevation,
        std::vector<uint8_t>& overrides);

    // destructor
    ~Map();

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
    std::pair<double, std::vector<bool>> shortestPath(pair src, pair dst);

    // check whether a point belongs to the map
    bool isValid(pair point) const;

    // check whether a point is an obstacle (water, marsh)
    bool isObstacle(pair point) const;

    // calculate the cost between two adjacent points
    double evalCost(pair src, pair dst) const;

    // calculate the different of elevation between two points
    int elevationDiff(pair src, pair dst) const;

    // get width of the map
    size_t width() const;

    // get height of the map
    size_t height() const;

    // get number of points
    size_t size() const;

    // get the index when the 2d array is flatten to 1D
    size_t getIndex(pair point) const;

};

}  // namespace mmap

#endif //BACHELOR_MAP_H
