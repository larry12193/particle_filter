/** @file particle.h
 *  @brief
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#ifndef _PARTICLE_COMMON_H
#define _PARTICLE_COMMON_H

#include <random>

struct particle_t {
    double x;
    double y;
    double theta;
    double weight;
} __attribute__((packed));

struct GaussianNoiseModel_t {
    double mean;
    double std;
    std::normal_distribution<double>* gaussian;
};

struct ExponentialNoiseModel_t {
    double lambda;
    std::exponential_distribution<double>* exp;
};

struct OdometryModel_t {
    double a1;
    double a2;
    double a3;
    double a4;
    GaussianNoiseModel_t transModel;
    GaussianNoiseModel_t rotModel;
};

struct LaserModel_t {
    double nhit;
    double hitFactor;
    GaussianNoiseModel_t hit;

    double nshrt;
    double shrtFactor;
    ExponentialNoiseModel_t shrt;

    double max;
    double maxFactor;

    double rand;
    double randFactor;
};

#endif /* _PARTICLE_COMMON_H */
