/** @file particle.h
 *  @brief
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#ifndef _PARTICLE_H
#define _PARTICLE_H

struct particle_t {
    double x;
    double y;
    double theta;
    double weight;
} __attribute__((packed));


#endif /* _PARTICLE_H */
