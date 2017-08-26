//
// Created by jun on 8/26/17.
//

#ifndef BACHELOR_EXPLORE_MAP_H
#define BACHELOR_EXPLORE_MAP_H

#include "../map.h"


/**
 * Find the maximum difference in elevation between two walkable grids
 */
void maxElevationDiff(const mmap::Map& map)
{
    typedef std::pair<size_t, size_t> pair;

    int max_diff = 0;
    for ( size_t k=0; k<map.width(); ++k )
    {
        for (size_t l=0; l<map.height(); ++l )
        {
            pair pick = std::make_pair(k, l);
            // loop over the surrounding 8 grids
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    pair pts = std::make_pair(pick.first + i, pick.second + j);
                    if (pts != pick && map.isValid(pts) && !map.isObstacle(pts)) {
                        int elevation_diff = std::abs(map.elevationDiff(pick, pts));
                        if ( elevation_diff > max_diff ) { max_diff = elevation_diff; }
                    }
                }
            }
        }
    }

    std::cout << "The maximum difference of elevation between two adjacent grids is: "
              << max_diff << std::endl;
}

#endif //BACHELOR_EXPLORE_MAP_H
