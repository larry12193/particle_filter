/** @file visualizer.h
 *  @brief Allows for visualization of particle filter solution
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#ifndef _VISUALIZER_H
#define _VISUALIZER_H

#include "robotdata.h"
#include "particle.h"

#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <boost/thread/thread.hpp>

#define UNMAPPED_R 0
#define UNMAPPED_G 0
#define UNMAPPED_B 255

#define OCCUPIED_R 255
#define OCCUPIED_G 255
#define OCCUPIED_B 255

static const std::string map_data = "/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/wean2.dat";
static const std::string arrow_file = "/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/arrow.png";

class Visualizer {

public:
    Visualizer();
    ~Visualizer();

    void drawParticles(particle_t *X);
    void viewImage();
private:

    void loadMap();
    void renderThread();

private:
    sf::RenderWindow* window;
    sf::Texture*      mapTexture;
    sf::Sprite*       mapSprite;
    sf::Uint8*        mapPixels;

    sf::Texture*      arrowTexture;
    sf::Sprite        arrowSprites[NUM_PARTICLES];

    sf::Thread*       figThread;
};

#endif /* _VISUALIZER_H */
