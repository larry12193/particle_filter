/** @file robotdata.h
 *  @brief Parameters for robotdata data sets.
 *
 *  This contains all the parameters pertaining to the robotdata data sets used
 *  to evaluate particle filter implementation
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */


#ifndef _ROBOTDATA_H
#define _ROBOTDATA_H

#include <string>

#define PI     3.14159265359
#define PI2    6.28318530718
#define ROOT2  1.41421356237
#define CM_TO_METERS  0.01
#define METER_TO_CM   100
#define TO_DEGREES    180.0/PI
#define TO_RADIANS    PI/180.0

/**
 * @brief Scale factor for arrow
 */
#define ARROW_SCALE_FACTOR 0.035

/**
 * @brief Pixel offset for arrow image
 */
#define ARROW_PIXEL_OFFSET 11

/**
 * @brief number of particles
 */
#define NUM_PARTICLES 2000

/**
 * @brief Pixel size resolution (cm)
 */
#define PIXEL_RESOLUTION 10

/**
 * @brief Map pixel height
 */
#define MAP_HEIGHT 800

/**
 * @brief Map pixel width
 */
#define MAP_WIDTH  800

/**
 * @brief Arrow pixel height
 */
#define ARROW_HEIGHT 600

/**
 * @brief Arrow pixel width
 */
#define ARROW_WIDTH  600

/**
 * @brief Pre-calculated number of open pixels on the map
 */
#define NUM_FREE_PARTICLES 31284

/**
 * @brief Data log delimeter
 */
static const std::string delimeter = " ";
/**
 * @brief Number of measurements in each laser scan
 */
#define NUM_MEAS 180

/**
 * @brief Laser X offset forward from center of robot (cm)
 */
#define LASER_OFFSET_X 25

/**
 * @brief Laser Y offset forward from center of robot (m)
 */
#define LASER_OFFSET_Y 0

/**
 * @brief Probabilistic value associated with occupied, unoccupied, and
 *        unmapped values in the map
 */
#define UNOCCUPIED  1.0
#define OCCUPIED    0
#define UNMAPPED   -1.0

/**
 * @brief Odometry entry column headers
 */
#define ODOMETRY_DATA_X       1
#define ODOMETRY_DATA_Y       2
#define ODOMETRY_DATA_HEADING 3
#define ODOMETRY_DATA_TIME    4

/**
 * @brief Laser scan entry column headers.
 */
#define LASER_ROBOT_X        1
#define LASER_ROBOT_Y        2
#define LASER_ROBOT_HEADING  3
#define LASER_ORIGIN_X       4
#define LASER_ORIGIN_Y       5
#define LASER_ORIGIN_HEADING 6
#define LASER_RANGE_START    7
#define LASER_RANGE_END      186
#define LASER_DATA_TIME      187
#define LASER_DTHETA         1

#endif /* _ROBOTDATA_H */
