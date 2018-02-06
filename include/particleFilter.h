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

#define NUM_PARTICLES 100

struct particle_t {
    double x;
    double y;
    double theta;
    double weight;
} __attribute__((packed));

class ParticleFilter {

public:
    ParticleFilter(std::string dataPath, std::string outputPath);
    ~ParticleFilter();

    void solve();

private:

    void motionModel(particle_t *particle);
    void resampleParticles();

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
};

 #endif /* _PARTICLE_FILTER_H */
