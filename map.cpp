//
// Created by jun on 8/24/17.
//

#include <cmath>
#include <vector>

#include "map.h"
#include "search.h"
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

std::pair<double, std::vector<bool>>
mmap::Map::shortestPath(pair src, pair dst)
{
    if (! isValid(src))
    {
        throw std::invalid_argument("Source point is not valid!");
    }

    if (! isValid(dst))
    {
        throw std::invalid_argument("destination point is not valid!");
    }

    if (isObstacle(src))
    {
        throw std::invalid_argument("Source point is unreachable!");
    }

    if (isObstacle(dst))
    {
        throw std::invalid_argument("destination point is unreachable!");
    }

    return msearch::aStarSearch(*this, src, dst);
};

bool mmap::Map::isValid(pair point) const
{
    return ( point.first >= 0 && point.first < width_ &&
             point.second >= 0 && point.second < height_ );
}

bool mmap::Map::isObstacle(pair point) const
{
    size_t idx = point.second*width_ + point.first;
    return (((*overrides_)[idx] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
            (*elevation_)[idx] == 0);
}

double mmap::Map::evalCost(pair src, pair dst) const
{
    double dist = 1.0;
    if ( src.first != dst.first && src.second != dst.second )
    {
        dist = 1.414213562;
    }

    int elevation_diff = elevationDiff(src, dst);
    if ( std::abs(elevation_diff) > 10 )
    {
        throw std::invalid_argument("Elevation difference cannot exceed 10!");
    }

    double cost;
    // The cost of one uphill point plus two downhill points equals to
    // that of three flat points. The difference in penalty/award reflects
    // the path length increase when going uphill or downhill.
    if (elevation_diff > 0)
    {
        cost = dist*(1.0 + elevation_diff/10.0);
    } else {
        cost = dist*(1.0 - elevation_diff/20.0);
    }

    return cost;
}

size_t mmap::Map::getIndex(pair point) const
{
    return (point.second*width_ + point.first);
}

int mmap::Map::elevationDiff(pair src, pair dst) const
{
    size_t src_idx = getIndex(src);
    size_t dst_idx = getIndex(dst);

    auto src_elevation = (int)(*elevation_)[src_idx];
    auto dst_elevation = (int)(*elevation_)[dst_idx];

    return (dst_elevation - src_elevation);
}


size_t mmap::Map::width() const { return width_; }

size_t mmap::Map::height() const { return height_; }

size_t mmap::Map::size() const { return width_*height_; }