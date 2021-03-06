#include "visualizer.h"
#include <iostream>
#include <fstream>
#include <algorithm>

#include "map.h"

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image

    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};

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
    printf("%s\n", argv[0]);

    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    auto elevation = loadFile("assets/elevation.data", expectedFileSize);
    auto overrides = loadFile("assets/overrides.data", expectedFileSize);

    // simplify overrides to a vector indicating obstacle or non-obstacle
    std::vector<bool> obstacles(overrides.size(), false);
    for ( size_t i=0; i < overrides.size(); ++i)
    {
        if ((overrides[i] & (OF_WATER_BASIN | OF_RIVER_MARSH)) || elevation[i] == 0)
        {
            obstacles[i] = true;
        }
    }

    mmap::Map map(IMAGE_DIM, IMAGE_DIM, elevation, obstacles);

    typedef std::pair<size_t, size_t> pair;
    pair rover = std::make_pair(ROVER_X, ROVER_Y);
    pair bachelor = std::make_pair(BACHELOR_X, BACHELOR_Y);
    pair wedding = std::make_pair(WEDDING_X, WEDDING_Y);

    clock_t t0 = clock();

    std::pair<double, std::vector<bool>> rover_to_bachelor;
    bool found_bachelor = false;
    std::cout << "Searching path from rover to bachelor...\n";
    try
    {
        // shortest path from rover to bachelor
        rover_to_bachelor = map.shortestPath(rover, bachelor);
        found_bachelor = true;
    }
    catch (const std::exception& ia)
    {
        std::cerr << ia.what() << std::endl;
    }

    std::pair<double, std::vector<bool>> bachelor_to_wedding;
    bool found_wedding = false;
    std::cout << "Searching path from bachelor to wedding...\n";
    try
    {
        // shortest path from bachelor to wedding
        bachelor_to_wedding = map.shortestPath(bachelor, wedding);
        found_wedding = true;
    }
    catch (const std::exception& ia)
    {
        std::cerr << ia.what() << std::endl;
    }

    std::cout << "Total search time: "
              << 1000.0*(clock() - t0)/CLOCKS_PER_SEC << " ms" << std::endl;

    std::ofstream of("pic.bmp");
    
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation)
        {
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            // Mark the path if found
            if (found_bachelor && rover_to_bachelor.second[y * IMAGE_DIM + x])
            {
                return uint8_t(visualizer::IPV_PATH);
            }
            if (found_wedding && bachelor_to_wedding.second[y * IMAGE_DIM + x])
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

    #ifdef __unix__
        std::cout << "Image file pic.bmp saved!" << std::endl;
    #else
        system("open pic.bmp");
    #endif

    return 0;
}
