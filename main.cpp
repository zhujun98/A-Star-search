#include "visualizer.h"
#include <iostream>
#include <fstream>
#include <assert.h>
#include <algorithm>

#include "map.h"
#include "test/test_map.h"
//#include "test/explore_map.h"
#include "parameters.h"


/**
 * Get the file size
 *
 * @param filename: path of the file
 * @return: file size
 */
std::ifstream::pos_type fileSize(const char* filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg(); 
}

/**
 * Load data from file.
 *
 * @param fileName: path of the file
 * @param expectedFileSize: the correct file size
 * @return: file data
 */
std::vector<uint8_t> loadFile(const char* fileName, size_t expectedFileSize)
{
    size_t fsize = fileSize(fileName);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(fileName, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

/**
 * Check whether a point is in a donut with the given center
 *
 * @param x: test coordinate x
 * @param y: test coordinate y
 * @param x1: donut center coordinate x
 * @param y1: donut center coordinate y
 * @return: true for in the donut
 */
bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}

int main(int argc, char** argv)
{
    testMapSearch();

    printf("%s\n", argv[0]);
    
    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    auto elevation = loadFile("assets/elevation.data", expectedFileSize);
    auto overrides = loadFile("assets/overrides.data", expectedFileSize);

    mmap::Map map(IMAGE_DIM, IMAGE_DIM, elevation, overrides);
//    maxElevationDiff(map);

    msearch::pair rover = std::make_pair(ROVER_X, ROVER_Y);
    msearch::pair bachelor = std::make_pair(BACHELOR_X, BACHELOR_Y);
    msearch::pair wedding = std::make_pair(WEDDING_X, WEDDING_Y);

    // shortest path from rover to bachelor
    std::pair<double, std::vector<bool>>
        roverToBachelor = msearch::shortestPath(map, rover, bachelor);

    std::cout << "The minimum time from the rover to the bachelor is: "
              << roverToBachelor.first << std::endl;

    // shortest path from bachelor to wedding
    std::pair<double, std::vector<bool>>
        bachelorToWedding = msearch::shortestPath(map, bachelor, wedding);

    std::cout << "The minimum time from the bachelor to the wedding is: "
              << bachelorToWedding.first << std::endl;

    std::ofstream of("pic.bmp");
    
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
        
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            // Mark the path
            if (roverToBachelor.second[y * IMAGE_DIM + x] ||
                bachelorToWedding.second[y * IMAGE_DIM + x])
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            // Signifies water
            if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                elevation == 0)
            {
                return uint8_t(visualizer::IPV_WATER);
            }

            // Signifies normal ground color
            if (elevation < visualizer::IPV_ELEVATION_BEGIN)
            {
                elevation = visualizer::IPV_ELEVATION_BEGIN;
            }
            return elevation;
    });
    system("open pic.bmp");
    return 0;
}
