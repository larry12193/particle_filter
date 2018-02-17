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
#include "particleCommon.h"
#include "matplotlibcpp.h"

#include <fstream>
#include <cstdlib>
#include <inttypes.h>
#include <ctime>
#include <random>
#include <cmath>
#include <math.h>
#include <vector>
#include <time.h>

static const std::string open_data = "/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/open_map.dat";
static const std::string noise_param = "/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/noiseData.dat";

#define NUM_BINS          100
#define SEARCH_DIST       0.5
#define RANGE_ANGLE_DELTA 10
#define RANDOM_PARTICLES  50
// #define DEBUG

class ParticleFilter {

public:
    ParticleFilter(std::string dataPath, std::string outputPath);
    ~ParticleFilter();

    void solve();

    void generateParticles();
    void plotRawTrajectory();
    void plotNoiseTrajectory();

private:

    void initializeRNG();

    void initializeModels();

    void loadLogicalMap();

    int spaceUnoccupied(double x, double y);

    /**
     * @brief Applies motion model to the
     */
    void applyMotionModel();

    void lowVarResampleParticles();

    void reinitializeOdometryGaussians(double r1, double r2, double tr);

    void findImportanceWeights();

    double rayTrace(particle_t p, double angle);

    uint16_t generateEmptyParticleSample();

    double generateThetaSample();

    void calculateLaserNomralizers();

    double integrateGaussian(double* zstar);
    double evaluateGaussian(double eval);

    double generateLaserSample();
    double laserModelProb(double zmeas);

    void visualizeLaserModel();
    double sample(double b);

private:

    /**
     * @brief Array of resampled particles
     */
    particle_t         _X[NUM_PARTICLES];
    particle_t         _lastX[NUM_PARTICLES];

    /**
     * @brief Array of resampled particles and their weights
     */
    particle_t         _Xbar[NUM_PARTICLES];

    std::string        _dataPath;
    std::string        _outputPath;

    LogReader*         _dataReader;
    dataFrame_t        _data;
    dataFrame_t        _lastData;

    Visualizer*        _visualizer;

    std::ifstream      _openFD;

    std::default_random_engine*              _rngen;
    std::uniform_real_distribution<double>*  _distUni;
    std::uniform_real_distribution<double>*  _distSamp;
    OdometryModel_t    _odomModel;
    LaserModel_t       _laserModel;

    bool _firstMeasurement;

    double _logicMap[MAP_HEIGHT][MAP_WIDTH];

    double _x[NUM_BINS+1];
    double _y[NUM_BINS+1];
};

 #endif /* _PARTICLE_FILTER_H */
