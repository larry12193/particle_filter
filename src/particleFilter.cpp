/** @file particleFilter.cpp
 *  @brief Implements particle filter localization algorithm for determining
 *         a robot's position via non-parametric Monte Carlo methods
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#include "particleFilter.h"

ParticleFilter::ParticleFilter(std::string dataPath, std::string outputPath) {

    _dataPath = dataPath;
    _outputPath = outputPath;

    // Initialize log file reader
    dataReader = new LogReader(dataPath);
    // dataReader.increment(&currentData);
    // dataReader.increment(&currentData);

    // Initialize visualizer
    visualizer = new Visualizer();

    // Seed the PRNG with system clock
    srand(static_cast<unsigned int>(time(0)));
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::solve() {

}

void ParticleFilter::generateParticles() {

    size_t pos = 0;
    uint16_t sampleRow;
    std::string line, token;
    bool dataValid;

    _openFD.open(open_data.c_str(), std::ifstream::in);

    // Check to make sure file was successfully opened
    if( !_openFD.good() ) {
        perror("Error opening open spaces data file.");
    }

    // Generate particles
    for( int i = 0; i < NUM_PARTICLES; i++ ) {
        // Grab row sample
        sampleRow = generateEmptyParticleSample();

        _openFD.seekg(0, _openFD.beg);

        // Get to row
        for( int j = 0; j < sampleRow; j++ ) {
            getline(_openFD, line);
        }

        // Find position of delimeter
        pos = line.find(delimeter);
        // Extract substring bounded by delimeter
        token = line.substr(0,pos);
        // Load data into current memory location
        X[i].x = std::atof(token.c_str()) * PIXEL_RESOLUTION;

        // Remove data from string
        line.erase(0,pos+delimeter.size());

        // Load data into current memory location
        X[i].y = std::atof(line.c_str()) * PIXEL_RESOLUTION;

        // Generate theta estimate
        X[i].theta = generateThetaSample();

        std::cout << "Particle #" << sampleRow;
        std::cout << " X: " << X[i].x << " Y: " << X[i].y << " T: " << X[i].theta << std::endl;
    }

    visualizer->drawParticles(&X[0]);
    visualizer->viewImage();
}

uint16_t ParticleFilter::generateEmptyParticleSample() {

    static const double fraction = 1.0 / (static_cast<double>(RAND_MAX) + 1.0);  // static used for efficiency, so we only calculate this value once
    // evenly distribute the random number across our range
    return static_cast<uint16_t>((NUM_FREE) * (rand() * fraction));
}

double ParticleFilter::generateThetaSample() {
    static const double fraction = 1.0 / (static_cast<double>(RAND_MAX) + 1.0);  // static used for efficiency, so we only calculate this value once
    // evenly distribute the random number across our range
    return static_cast<double>((360 + 1) * (rand() * fraction));
}
