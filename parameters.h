//
// Global parameters.
//

#ifndef BACHELOR_PARAMETERS_H
#define BACHELOR_PARAMETERS_H

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

#endif //BACHELOR_PARAMETERS_H
