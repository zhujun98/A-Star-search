//
// Created by jun on 8/24/17.
//

#include <cmath>
#include <vector>

#include "map.h"
#include "parameters.h"


mmap::Map::Map(size_t width,
         size_t height,
         std::vector<uint8_t>& elevation,
         std::vector<uint8_t>& overrides) :
    width_(width),
    height_(height),
    elevation_(&elevation),
    overrides_(&overrides) {}

mmap::Map::~Map() {}

bool mmap::Map::isValid(pair grid) const
{
    return ( grid.first >= 0 && grid.first < width_ &&
             grid.second >= 0 && grid.second < height_ );
}

bool mmap::Map::isObstacle(pair grid) const
{
    size_t idx = grid.second*width_ + grid.first;
    return (((*overrides_)[idx] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
            (*elevation_)[idx] == 0);
}

double mmap::Map::evalCost(pair src, pair dst) const
{
    size_t src_idx = getIndex(src);
    size_t dst_idx = getIndex(dst);

    double dx = (double)src.first - (double)dst.first;
    double dy = (double)src.second - (double)dst.second;

    double cost = std::sqrt(dx*dx + dy*dy);

    return cost;
}

size_t mmap::Map::getIndex(pair grid) const
{
    return (grid.second*width_ + grid.first);
}

size_t mmap::Map::width() const { return width_; }

size_t mmap::Map::height() const { return height_; }

size_t mmap::Map::size() const { return width_*height_; }