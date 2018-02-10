/** @file visualizer.cpp
 *  @brief Allows for visualization of particle filter solution
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */


#include "visualizer.h"
#include "robotdata.h"

Visualizer::Visualizer() {

    window = new sf::RenderWindow(sf::VideoMode(MAP_WIDTH, MAP_HEIGHT), "Particle Filter");
    mapTexture = new sf::Texture;
    mapTexture->create(MAP_WIDTH, MAP_HEIGHT);
    mapSprite  = new sf::Sprite;
    figThread = new sf::Thread(&Visualizer::renderThread,this);

    arrowTexture = new sf::Texture;
    arrowTexture->loadFromFile(arrow_file);

    // Load map and interpret as pixel data
    loadMap();
}

Visualizer::~Visualizer() {

}

void Visualizer::loadMap() {

    // Define pixel value array
    mapPixels = new sf::Uint8[MAP_HEIGHT * MAP_WIDTH * 4];

    bool dataValid = true;
    double pixelData;
    char greyVal;
    size_t pos = 0;
    std::string line, token;
    std::ifstream fd;
    fd.open(map_data.c_str(), std::ifstream::in);

    // Check to make sure file was successfully opened
    if( !fd.good() ) {
        perror("Error opening map data file.");
    }

    // Keeps track of what pixel is being set
    int pixelCount = 0;

    // Iterate over every line in the map data file
    while( std::getline(fd,line) ) {
        // Iterate over all delimeted elements
        while( dataValid ) {
            // Find position of next delimeter
            pos = line.find(delimeter);
            // Check if we reached end of line
            if( pos == std::string::npos ) {
                dataValid = false;
            }

            // Extract substring bounded by delimeter
            token = line.substr(0,pos);

            // Load data into current memory location
            pixelData = std::atof(token.c_str());

            // Handle each kind of pixel value
            if( pixelData == OCCUPIED ) {
                mapPixels[pixelCount++] = OCCUPIED_R;
                mapPixels[pixelCount++] = OCCUPIED_G;
                mapPixels[pixelCount++] = OCCUPIED_B;
                mapPixels[pixelCount++] = 255;
            } else if( pixelData == UNMAPPED ) {
                mapPixels[pixelCount++] = UNMAPPED_R;
                mapPixels[pixelCount++] = UNMAPPED_G;
                mapPixels[pixelCount++] = UNMAPPED_B;
                mapPixels[pixelCount++] = 255;
            } else {
                greyVal = (char)(255.0*pixelData);
                mapPixels[pixelCount++] = greyVal;
                mapPixels[pixelCount++] = greyVal;
                mapPixels[pixelCount++] = greyVal;
                mapPixels[pixelCount++] = 255;
            }

            // Remove data from string
            line.erase(0,pos+delimeter.size());
        }
        // Reset flag for next line
        dataValid = true;
    }

    // Load the map into texture
    mapTexture->update(mapPixels);
    // Load texture into sprite
    mapSprite->setTexture(*mapTexture);
    //boost::thread thread(boost::bind(&Visualizer::window_thread,this));
    // Join thread
    //thread.join();
}

void Visualizer::renderThread() {
    while( window->isOpen()) {
        window->draw(*mapSprite);
        for( int i = 0; i < NUM_PARTICLES; i++ ) {
            window->draw(arrowSprites[i]);
        }
        window->display();
    }
}

void Visualizer::drawParticles(particle_t *X) {
    for( int i = 0; i < NUM_PARTICLES; i++ ) {
        arrowSprites[i].setTexture(*arrowTexture);
        arrowSprites[i].setOrigin(300,300);
        arrowSprites[i].setRotation(X->theta);
        arrowSprites[i].setPosition(X->y/PIXEL_RESOLUTION,X->x/PIXEL_RESOLUTION);
        arrowSprites[i].setScale(0.035,0.035);
        X++;
    }
}

void Visualizer::viewImage() {
    // Create thread to update the window
    figThread->launch();
}
