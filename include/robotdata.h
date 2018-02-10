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

#define PI  3.14159265359
#define PI2 6.28318530718

#define NUM_PARTICLES 100

/**
 * @brief Pixel size resolution (cm)
 */
#define PIXEL_RESOLUTION 10

#define MAP_HEIGHT 800
#define MAP_WIDTH  800

#define ARROW_WIDTH  600
#define ARROW_HEIGHT 600

#define NUM_FREE 31284

/**
 * @brief Data log delimeter
 */
static const std::string delimeter = " ";
/**
 * @brief Number of measurements in each laser scan
 */
#define NUM_MEAS 180

/**
 * @brief Laser offset forward from center of robot (cm)
 */
#define LASER_OFFSET_X 25
#define LASER_OFFSET_Y 0

/**
 * @brief Probabilistic value associated with occupied, unoccupied, and
 *        unmapped values in the map
 */
#define OCCUPIED 1.0
#define UNOCCUPIED 0
#define UNMAPPED -1.0

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
