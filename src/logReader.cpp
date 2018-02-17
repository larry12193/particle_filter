/** @file logReader.cpp
 *  @brief Method definition for logReader class
 *
 *  This contains the class definition for a class that is able to read the
 *  robotdata log files and return the appropriate form of data
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#include "logReader.h"

LogReader::LogReader(std::string logFile) {

    _logFile = logFile;
    // Attempt to open file
    _fd.open(logFile.c_str(), std::ifstream::in);

    // Check to make sure file was successfully opened
    if( !_fd.good() ) {
        perror("Error opening data file.");
    }
}

LogReader::~LogReader() {
    if( _fd.is_open() ) {
        _fd.close();
    }
}

int LogReader::increment(dataFrame_t *dataFrame) {

    // Get next line in file
    if( !_fd.eof() ) {
        std::getline(_fd,_line);
        if( decodeDataLine(dataFrame) < 0 ) {
            return -1;
        };
        return 1;
    } else {
        return -1;
    }
}

int LogReader::decodeDataLine(dataFrame_t *dataFrame) {

    // Process data by line identifier
    if( _line[0] == 'L' ) {
        _line.erase(0,delimeter.size() + 1);
        dataFrame->type = laserScanData;
    } else if( _line[0] == 'O' ) {
        _line.erase(0,delimeter.size() + 1);
        dataFrame->type = odometryData;
    } else {
        return -1;
        //perror("Encountered unknown data entry in log file, aborting.");
    }
    lineDecoder(dataFrame);
    return 1;
}

void LogReader::lineDecoder(dataFrame_t *dataFrame) {

    size_t pos = 0;
    std::string token;

    double *data;
    if( dataFrame->type == odometryData ) {
        data = &dataFrame->odomData.robotX;
    } else {
        data = &dataFrame->laserData.robotX;
    }

    bool dataValid = true;

    while( dataValid ) {
        // Find position of next delimeter
        pos = _line.find(delimeter);
        // Check if we reached end of line
        if( pos == std::string::npos ) {
            dataValid = false;
        }
        // Extract substring bounded by delimeter
        token = _line.substr(0,pos);

        // Load data into current memory location
        (*data) = std::atof(token.c_str());
        // Move to next memory location
        data++;

        // Remove data from string
        _line.erase(0,pos+delimeter.size());
    }

    // Print out extracted data for verification
    // if( dataFrame->type == odometryData ) {
    //     std::cout << "Odometry frame:" << std::endl;
    //     std::cout << "Robot X = " << dataFrame->odomData.robotX << std::endl;
    //     std::cout << "Robot Y = " << dataFrame->odomData.robotY << std::endl;
    //     std::cout << "Robot Theta = " << dataFrame->odomData.robotTheta << std::endl;
    //     std::cout << "Time = " << dataFrame->odomData.time << std::endl;
    // } else {
    //     std::cout << "Laser Scan frame:" << std::endl;
    //     std::cout << "Robot X = " << dataFrame->laserData.robotX << std::endl;
    //     std::cout << "Robot Y = " << dataFrame->laserData.robotY << std::endl;
    //     std::cout << "Robot Theta = " << dataFrame->laserData.robotTheta << std::endl;
    //     std::cout << "Laser X = " << dataFrame->laserData.laserX << std::endl;
    //     std::cout << "Laser Y = " << dataFrame->laserData.laserY << std::endl;
    //     std::cout << "Laser Theta = " << dataFrame->laserData.laserTheta << std::endl;
    //     for(int i = 0; i < 180; i++ ) {
    //         std::cout << "Range #" << i << " " << dataFrame->laserData.ranges[i] << std::endl;
    //     }
    //     std::cout << "Time = "<< dataFrame->laserData.time << std::endl;
    // }
    // getchar();
}
