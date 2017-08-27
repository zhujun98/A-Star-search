//
// Test Map class and A* search.
//

#include <iostream>
#include <assert.h>
#include <algorithm>
#include <vector>
#include "../map.h"


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
    std::vector<bool> obstacles(width*height, false);

    mmap::Map map(width, height, elevation, obstacles);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(3, 1), 1, 1, 0);

//    display(path, width);

    assert(std::abs(path.first - 3.41421) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 4);
    std::cout << "Passed!" << std::endl;
}


/**
 *  Run test on a map with river marsh and water basin
 *
 *   1  O  1  1
 *   1  O  O  1
 *   1  1  1  1
 */
void testWaterMap()
{
    size_t width = 4;
    size_t height = 3;
    std::vector<uint8_t> elevation(width*height, 0x01);
    std::vector<bool> obstacles(width*height, false);

    obstacles[1] = true;
    obstacles[5] = true;
    obstacles[6] = true;

    mmap::Map map(width, height, elevation, obstacles);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(3, 1), 1, 1, 0);

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

    std::vector<bool> obstacles {0, 0, 0, 0, 0, 0,
                                 0, 1, 0, 1, 1, 0,
                                 0, 0, 0, 1, 1, 0,
                                 0, 0, 0, 0, 0, 0};

    mmap::Map map(width, height, elevation, obstacles);

    std::pair<double, std::vector<bool>>
        path = map.shortestPath(std::make_pair(0, 0), std::make_pair(5, 3), 1, 1, 0);

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
