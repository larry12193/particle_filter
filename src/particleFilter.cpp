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
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::solve() {

}
