//
// Map class.
//

#include <cmath>
#include <vector>

#include "map.h"
#include "search.h"
#include "parameters.h"


mmap::Map::Map(size_t width,
         size_t height,
         const std::vector<uint8_t>& elevation,
         const std::vector<uint8_t>& overrides) :
    width_(width),
    height_(height),
    elevation_(&elevation),
    overrides_(&overrides)
{
//    maxElevationDiff();
}

mmap::Map::~Map() {}

bool mmap::Map::isValid(const pair& point) const
{
    return ( point.first >= 0 && point.first < width_ &&
             point.second >= 0 && point.second < height_ );
}

bool mmap::Map::isObstacle(const pair& point) const
{
    size_t idx = point.second*width_ + point.first;

    return (((*overrides_)[idx] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
            (*elevation_)[idx] == 0);
}

std::deque<mmap::pair> mmap::Map::neighbors(const pair& src) const
{
    std::deque<pair> neighbors;
    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            pair pts = std::make_pair(src.first + i, src.second + j);
            if (pts != src && isValid(pts) && !isObstacle(pts))
            {
                neighbors.push_back(pts);
            }
        }
    }

    return neighbors;
}

std::pair<double, std::vector<bool>>
mmap::Map::shortestPath(const pair& src, const pair& dst, bool fast_search) const
{
    if (! isValid(src))
    {
        throw std::out_of_range("Out of range: source!");
    }
    if (! isValid(dst))
    {
        throw std::out_of_range("Out of range: destination!");
    }

    if (isObstacle(src))
    {
        throw std::invalid_argument("Invalid_argument: source unreachable!");
    }
    if (isObstacle(dst))
    {
        throw std::invalid_argument("Invalid_argument: destination unreachable!");
    }

    auto result = msearch::aStarSearch(*this, src, dst, fast_search);

    std::cout << "The minimum time consumption: "
              << result.first << " island seconds" << std::endl;

    return result;
};

double mmap::Map::evalCost(const pair& src, const pair& dst) const
{
    double dist = 1.0;
    if ( src.first != dst.first && src.second != dst.second )
    {
        dist = 1.414213562;
    }

    int elevation_diff = elevationDiff(src, dst);
    if ( std::abs(elevation_diff) > 20 )
    {
        std::cerr << "The model may be inadequate when elevation difference "
                  << "is larger than 20! " << std::endl;
    }

    // The cost of one uphill point plus two downhill points equals to
    // that of three flat points. The difference in penalty/award
    // reflects the path length increase no matter if the rover goes
    // uphill or downhill.
    double cost;
    if (elevation_diff > 0)
    {
        cost = dist + dist*elevation_diff/10.0;
    } else {
        cost = dist - dist*elevation_diff/20.0;
    }

    return cost;
}

size_t mmap::Map::getIndex(const pair& point) const
{
    return (point.second*width_ + point.first);
}

int mmap::Map::elevationDiff(const pair& src, const pair& dst) const
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

void mmap::Map::maxElevationDiff() const
{
    typedef std::pair<size_t, size_t> pair;

    int max_diff = 0;
    for ( size_t k=0; k<width_; ++k )
    {
        for (size_t l=0; l<height_; ++l )
        {
            pair pick = std::make_pair(k, l);
            // loop over neighbor points
            for (const auto& pts : neighbors(pick))
            {
                if (pts != pick && isValid(pts) && !isObstacle(pts))
                {
                    int elevation_diff = std::abs(elevationDiff(pick, pts));
                    if ( elevation_diff > max_diff )
                    {
                        max_diff = elevation_diff;
                    }
                }
            }
        }
    }

    std::cout << "Maximum difference of elevation: " << max_diff << std::endl;
}