//
// Test Map class and A* search.
//

#include <iostream>
#include <assert.h>
#include <algorithm>
#include <vector>
#include "../map.h"

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

/**
 * Summarize the shorted path
 *
 * @param path: output from function shortestPath()
 * @param height: height of the map
 */
void display(std::pair<double, std::vector<bool>> path, size_t width)
{
    std::cout << "Shortest path length: " << path.first << std::endl;
    std::cout << "The shortest path is: " << std::endl;

    for (size_t i = 0; i < path.second.size(); ++i)
    {
        std::cout << path.second[i];
        if ( (i + 1) % width == 0 ) { std::cout << std::endl; }
    }
    std::cout << std::endl;
}

/**
 * Run test on a uniform path
 *
 *   1  1  1  1
 *   1  1  1  1
 *   1  1  1  1
 */
void testUniformMap()
{
    size_t width = 4;
    size_t height = 3;
    std::vector<uint8_t> elevation(width*height, 0x01);
    std::vector<uint8_t> overrides(width*height, 0x20);

    mmap::Map map(width, height, elevation, overrides,
                  OF_RIVER_MARSH, OF_WATER_BASIN);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(3, 1), 0, 0);

//    display(path, width);

    assert(std::abs(path.first - 3.41421) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 4);
    std::cout << "Passed!" << std::endl;
}


/**
 *  Run test on a map with river marsh and water basin
 *
 *   1  R  1  1
 *   1  W  R  1
 *   1  1  1  1
 */
void testWaterMap()
{
    size_t width = 4;
    size_t height = 3;
    std::vector<uint8_t> elevation(width*height, 0x01);
    std::vector<uint8_t> overrides(width*height, 0x20);

    overrides[1] = 0x10;
    overrides[5] = 0x40;
    overrides[6] = 0x10;

    mmap::Map map(width, height, elevation, overrides,
                  OF_RIVER_MARSH, OF_WATER_BASIN);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(3, 1), 0, 0);

    assert(std::abs(path.first - 4.82843) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 5);
    std::cout << "Passed!" << std::endl;
}

/**
 *  Run test on a map with river marsh and water basin as well as
 *  variable elevations.
 *
 *   0x01 0x02 0x02 0x04 0x08 0x04
 *   0x02    R 0x08    W    W 0x02
 *   0x04 0x02 0x01    W    W 0x01
 *   0x02 0x04 0x04 0x02 0x01 0x01
 */
void testFullMap()
{
    size_t width = 6;
    size_t height = 4;
    std::vector<uint8_t> elevation {0x01, 0x02, 0x02, 0x04, 0x08, 0x04,
                                    0x02, 0xFF, 0x08, 0xFF, 0xFF, 0x02,
                                    0x04, 0x02, 0x01, 0xFF, 0xFF, 0x01,
                                    0x02, 0x04, 0x04, 0x02, 0x01, 0x01};

    std::vector<uint8_t> overrides {0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
                                    0x20, 0x10, 0x20, 0x40, 0x40, 0x20,
                                    0x20, 0x20, 0x20, 0x40, 0x40, 0x20,
                                    0x20, 0x20, 0x20, 0x20, 0x20, 0x20};

    mmap::Map map(width, height, elevation, overrides,
                  OF_RIVER_MARSH, OF_WATER_BASIN);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(5, 3), 0, 0);

//    display(path, width);

    assert(std::abs(path.first - 7.16985) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 7);
    std::cout << "Passed!" << std::endl;
}

/**
 * Test path search in different maps
 */
int main()
{
    testUniformMap();
    testWaterMap();
    testFullMap();
}
