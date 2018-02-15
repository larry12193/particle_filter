/** @file visualizer.cpp
 *  @brief Allows for visualization of particle filter solution
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */


#include "visualizer.h"
#include "robotdata.h"

Visualizer::Visualizer() {

    _drawDot = false;
    window = new sf::RenderWindow(sf::VideoMode(MAP_WIDTH, MAP_HEIGHT), "Particle Filter");
    mapTexture = new sf::Texture;
    mapTexture->create(MAP_WIDTH, MAP_HEIGHT);
    mapSprite  = new sf::Sprite;
    figThread = new sf::Thread(&Visualizer::renderThread,this);

    arrowTexture = new sf::Texture;
    arrowTexture->loadFromFile(arrow_file);

    dotTexture = new sf::Texture;
    dotTexture->loadFromFile(dot_file);

    // Load map and interpret as pixel data
    loadMap();
}

Visualizer::~Visualizer() {

}

void Visualizer::loadMap() {

    // Define pixel value array ( RBGA based pixel information )
    mapPixels = new sf::Uint8[MAP_HEIGHT * MAP_WIDTH * 4];

    // Used to catch final delimited data in string
    bool dataValid = true;
    // Raw pixel value
    double pixelData;
    // Value of grey pixel when probability is not -1 or 1
    char greyVal;
    // Tracks position of delimiter in string being parsed
    size_t pos = 0;

    std::string line, token;
    std::ifstream fd;

    // Open map file
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

            } else if( pixelData == UNMAPPED ) {
                mapPixels[pixelCount++] = UNMAPPED_R;
                mapPixels[pixelCount++] = UNMAPPED_G;
                mapPixels[pixelCount++] = UNMAPPED_B;
            } else {
                greyVal = (char)(255.0*pixelData);
                mapPixels[pixelCount++] = greyVal;
                mapPixels[pixelCount++] = greyVal;
                mapPixels[pixelCount++] = greyVal;
            }
            // Set pixel alpha value
            mapPixels[pixelCount++] = 255;

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
}

void Visualizer::renderThread() {
    // Run while window is still open
    while( window->isOpen()) {
        // Draw the map
        window->draw(*mapSprite);
        // Draw all particles sprites
        for( int i = 0; i < NUM_PARTICLES; i++ ) {
            window->draw(arrowSprites[i]);
        }

        if( _drawDot ) {
            window->draw(*dotSprite);
        }

        // Display what is currently rendered
        window->display();

        while( window->pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window->close();
                break;
            }

            // Mouse button press event
            if (event.type == sf::Event::MouseButtonPressed) {
                std::cout << " X(" << event.mouseButton.x << ")";
                std::cout << " YZ(" << event.mouseButton.y << ")" << std::endl;
                break;
            }
        }
    }
}

void Visualizer::drawParticles(particle_t *X) {
    // Iterate over all particles and add in arrow sprite
    for( int i = 0; i < NUM_PARTICLES; i++ ) {
        arrowSprites[i].setTexture(*arrowTexture);
        arrowSprites[i].setOrigin(ARROW_WIDTH/2,ARROW_HEIGHT/2);
        arrowSprites[i].setRotation(X->theta*TO_DEGREES);
        arrowSprites[i].setPosition(X->x/PIXEL_RESOLUTION + 1.0,X->y/PIXEL_RESOLUTION + 1.0);
        arrowSprites[i].setScale(ARROW_SCALE_FACTOR,ARROW_SCALE_FACTOR);
        // Move to next particle
        X++;
    }
}

void Visualizer::flagPixel(double x, double y) {
    // sf::Uint8 p[4];
    // p[0] = 255;
    // p[1] = 0;
    // p[2] = 0;
    // p[3] = 255;
    int px, py;
    // Convert xy position to pixel value, correcting for pixels starting at (1,1)
    // which equates to (0.0,0.0) in world coordinates
    px = (x/PIXEL_RESOLUTION);
    py = (y/PIXEL_RESOLUTION);

    dotSprite = new sf::Sprite;
    dotSprite->setTexture(*dotTexture);
    dotSprite->setOrigin(100,100);
    dotSprite->setPosition(px,py);
    dotSprite->setScale(0.025,0.025);
    _drawDot = true;
    //(*mapTexture).update(&p[0],1,1,x,y);
}

void Visualizer::viewImage() {
    // Create thread to update the window
    figThread->launch();
}
