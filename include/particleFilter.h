/** @file particleFilter.h
 *  @brief Implements particle filter localization algorithm for determining
 *         a robot's position via non-parametric Monte Carlo methods
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

 #ifndef _PARTICLE_FILTER_H
 #define _PARTICLE_FILTER_H

#include "robotdata.h"
#include "logReader.h"
#include "visualizer.h"
#include "particle.h"

#include <fstream>
#include <cstdlib>
#include <inttypes.h>
#include <ctime>


static const std::string open_data = "/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/open_map.dat";

class ParticleFilter {

public:
    ParticleFilter(std::string dataPath, std::string outputPath);
    ~ParticleFilter();

    void solve();

    void generateParticles();

private:

    void motionModel(particle_t *particle);

    uint16_t generateEmptyParticleSample();
    double generateThetaSample();
private:

    /**
     * @brief Array of resampled particles
     */
    particle_t X[NUM_PARTICLES];
    /**
     * @brief Array of resampled particles and their weights
     */
    particle_t Xbar[NUM_PARTICLES];

    std::string _dataPath;
    std::string _outputPath;

    LogReader *dataReader;

    Visualizer *visualizer;

    std::ifstream _openFD;
};

 #endif /* _PARTICLE_FILTER_H */
