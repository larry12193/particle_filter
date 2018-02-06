/** @file logReader.h
 *  @brief Class definition of robotdata log file reader
 *
 *  This contains the class definition for a class that is able to read the
 *  robotdata log files and return the appropriate form of data
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#ifndef _LOG_READER_H
#define _LOG_READER_H

#include "robotdata.h"
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

enum dataType_t {
    odometryData,
    laserScanData,
};

struct laserDataFrame_t {
    double robotX;
    double robotY;
    double robotTheta;
    double laserX;
    double laserY;
    double laserTheta;
    double ranges[NUM_MEAS];
    double time;
}__attribute__((packed));

struct odometryDataFrame_t {
    double robotX;
    double robotY;
    double robotTheta;
    double time;
}__attribute__((packed));

struct dataFrame_t {
    dataType_t type;
    laserDataFrame_t laserData;
    odometryDataFrame_t odomData;
} __attribute__((packed));

class LogReader {

public:

    /** @brief Constructor for LogReader classs
     *
     *  @param logFile The location of the log file to be used
     *  @return Void.
     */
    LogReader(std::string logFile);
    
    /** @brief Deconstructor for LogReader classs
     *
     */
    ~LogReader();

    /** @brief Increments data to next entry
     *
     *  @return Success, end of file, or error.
     */
    uint8_t increment(dataFrame_t *dataFrame);

private:

    /** @brief Constructor for LogReader classs
     *
     *  @param logFile The location of the log file to be used
     *  @return Void.
     */
    void decodeDataLine(dataFrame_t *dataFrame);

    /** @brief Decodes a line of odometry data
     *
     *  @param line The line of odometry data to be decoded
     *  @return Void.
     */
     void lineDecoder(dataFrame_t *dataFrame);

private:

    std::ifstream _fd;
    std::string _logFile;
    std::string _line;

}; /* logReader */

 #endif /* _LOG_READER_H */
