//
// Created by jun on 8/24/17.
//

#ifndef BACHELOR_TEST_MAP_H
#define BACHELOR_TEST_MAP_H

#include <iostream>
#include <algorithm>
#include <vector>
#include "../map.h"
#include "../search.h"


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

    mmap::Map map(width, height, elevation, overrides);

    std::pair<double, std::vector<bool>>
        path = msearch::shortestPath(map, std::make_pair(0, 0), std::make_pair(3, 1));

    display(path, width);

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

    mmap::Map map(width, height, elevation, overrides);

    std::pair<double, std::vector<bool>>
        path = msearch::shortestPath(map, std::make_pair(0, 0), std::make_pair(3, 1));

    assert(std::abs(path.first - 4.82843) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 5);
    std::cout << "Passed!" << std::endl;
}

/**
 *  Run test on a map with river marsh and water basin as well as
 *  variable elevations.
 *
 *   0x01 0x01 0xFF 0x01
 *   0x24 0x10    R 0x01
 *   0x01    W 0x01 0x01
 */
void testFullMap()
{
    size_t width = 4;
    size_t height = 3;
    std::vector<uint8_t> elevation(width*height, 0x01);
    std::vector<uint8_t> overrides(width*height, 0x20);

    overrides[6] = 0x10;
    overrides[9] = 0x40;

    elevation[2] = 0xFF;
    elevation[4] = 0x24;
    elevation[5] = 0x10;

    mmap::Map map(width, height, elevation, overrides);

    std::pair<double, std::vector<bool>>
        path = msearch::shortestPath(map, std::make_pair(0, 0), std::make_pair(3, 1));

    //    display(path, width);

    assert(std::abs(path.first - 3.41421) < 1.0e-5 );
    assert(std::accumulate(path.second.begin(), path.second.end(), 0) == 4);
    std::cout << "Passed!" << std::endl;
}

#endif //BACHELOR_TEST_MAP_H
