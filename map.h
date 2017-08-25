//
// Created by jun on 8/24/17.
//

#ifndef BACHELOR_MAP_H
#define BACHELOR_MAP_H

#include <iostream>
#include <string>


namespace mmap {

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

    // check whether a grid belongs to the map
    bool isValid(pair grid) const;

    // check whether a grid is an obstacle (water, marsh)
    bool isObstacle(pair grid) const;

    // calculate the cost between two grids
    double evalCost(pair src, pair dst) const;

    // get width of the map
    size_t width() const;

    // get height of the map
    size_t height() const;

    // get number of grids
    size_t size() const;
};

}  // namespace mmap

#endif //BACHELOR_MAP_H
