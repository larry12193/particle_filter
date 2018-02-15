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
    _dataReader = new LogReader(dataPath);

    // Initialize _visualizer
    _visualizer = new Visualizer();

    // Initialize the random number generators and sample distrobutions
    this->initializeRNG();

    // Intialize the motion model
    this->initializeModels();

    // Load in logical map used for ray tracing
    this->loadLogicalMap();

    // Generate inital particle set
    this->generateParticles();
    // _X[0].y = 61.8;
    // _X[0].x = 65.4;
    // _X[0].theta = 0.0;
    // _visualizer->drawParticles(&_X[0]);
    // _visualizer->viewImage();

    // Flag first measurement
     _firstMeasurement = true;
}

ParticleFilter::~ParticleFilter() {
    delete _visualizer;
    delete _dataReader;
    _openFD.close();
}

void ParticleFilter::solve() {

    //Run over entire data set
    while( _dataReader->increment(&_data) > 0 ) {
        if( _data.type == laserScanData ) {

            if( _firstMeasurement ) {
                _lastData = _data;
                _firstMeasurement = false;
            } else {
                // Propogate points using odometry
                applyMotionModel();
                // Find importance weights based on sensor model
                findImportanceWeights();
    #ifdef DEBUG
                for( int i = 0; i < NUM_PARTICLES; i++ ) {
                    std::cout << "Xbar" << i <<  " weight = " << _Xbar[i].weight << std::endl;
                }
    #endif
                // Resample particles
                resampleParticles();
                _visualizer->drawParticles(&_X[0]);

                // Save last data frame for next update
                _lastData = _data;
            }
            getchar();
        }
    }
    std::cout << "Particle filter complete" << std::endl;
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
        _X[i].x = std::atof(token.c_str()) * PIXEL_RESOLUTION;

        // Remove data from string
        line.erase(0,pos+delimeter.size());

        // Load data into current memory location
        _X[i].y = std::atof(line.c_str()) * PIXEL_RESOLUTION;

        // Generate theta estimate
        _X[i].theta = generateThetaSample();

        // std::cout << "Particle #" << sampleRow;
        // std::cout << " _X: " << _X[i].x << " Y: " << _X[i].y << " T: " << _X[i].theta << std::endl;
    }
    _openFD.close();

    _visualizer->drawParticles(&_X[0]);
    _visualizer->viewImage();
}

void ParticleFilter::initializeRNG() {

    // Initialize PRNG with time seed
    _rngen = new std::default_random_engine(time(0));

    _distUni = new std::uniform_real_distribution<double>(0.0,1.0);
}

void ParticleFilter::initializeModels() {

    size_t pos = 0;
    uint16_t sampleRow;
    std::string line, token;
    bool dataValid;

    _openFD.open(noise_param.c_str(), std::ifstream::in);

    // Check to make sure file was successfully opened
    if( !_openFD.good() ) {
        perror("Error opening noise paramater data file.");
    }

    // Extract translational model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a1 = std::atof(token.c_str());

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a2 = std::atof(token.c_str());

    // Extract rotational model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a3 = std::atof(token.c_str());

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a4 = std::atof(token.c_str());

    // Extract laser hit model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.hit.mean = std::atof(token.c_str());

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.hit.std = std::atof(token.c_str());

    // Initialize rotational gaussian
    _laserModel.hit.gaussian = new std::normal_distribution<double>(_laserModel.hit.mean,_laserModel.hit.std);

    // Extract laser unexpected objects model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.shrt.lambda = std::atof(token.c_str());
    _laserModel.shrt.exp = new std::exponential_distribution<double>(_laserModel.shrt.lambda);

    // Extract laser max range
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.max = std::atof(token.c_str());

    // Extract laser rand range
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.rand = std::atof(token.c_str());

    // Extract laser hit factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.hitFactor = std::atof(token.c_str());

    // Extract laser short factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.shrtFactor = std::atof(token.c_str());

    // Extract laser max factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.maxFactor = std::atof(token.c_str());

    // Extract laser rand factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.randFactor = std::atof(token.c_str());

    _openFD.close();
}

void ParticleFilter::loadLogicalMap() {
    std::ifstream fd;
    fd.open(map_data.c_str(),std::ifstream::in);
    size_t pos = 0;
    std::string token, line;
    float* data = &_logicMap[0];

    bool dataValid = true;
    while( getline(fd,line) ) {
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
            (*data) = std::atof(token.c_str());
            // Move to next memory location
            data++;

            // Remove data from string
            line.erase(0,pos+delimeter.size());
        }
        dataValid = true;
    }
}

void ParticleFilter::applyMotionModel() {
    double r1,r2,tr;
    double r1h,r2h,trh;

    double dx  = _data.laserData.robotX - _lastData.laserData.robotX;
    double dy  = _data.laserData.robotY - _lastData.laserData.robotY;
    double dth = _data.laserData.robotTheta - _lastData.laserData.robotTheta;

    if( dx != 0.0 && dy != 0.0 && dth != 0 ) {
    #ifdef DEBUG
        std::cout << "dx = " << dx << " dy = " << dy << " dth = " << dth << std::endl;
        getchar();
    #endif

        memcpy(_lastX,_X,sizeof(_lastX));

    #ifdef DEBUG
        std::cout << "lX = " << _lastX[0].x << " lY = " << _lastX[0].y << " lTh = " << _lastX[0].theta << std::endl;
        getchar();
    #endif

        for( int i = 0; i < NUM_PARTICLES; i++ ) {

            _Xbar[i].x = _lastX[i].x + dx;
            _Xbar[i].y = _lastX[i].y + dy;
            _Xbar[i].theta = _lastX[i].theta + dth;

    #ifdef DEBUG
            std::cout << "bX = " << _Xbar[i].x << " bY = " << _Xbar[i].y << " bTh = " << _Xbar[i].theta << std::endl;
            getchar();
    #endif

            r1 = atan2(_Xbar[i].y - _lastX[i].y,_Xbar[i].x - _lastX[i].x) - _lastX[i].theta;
            dx = _lastX[i].x - _Xbar[i].x;
            dy = _lastX[i].y - _Xbar[i].y;
            tr = sqrt(dx*dx + dy*dy);
            r2 = _Xbar[i].theta - _lastX[i].theta - r1;

    #ifdef DEBUG
            std::cout << "r1 = " << r1 << " tr = " << tr << " r2 = " << r2 << std::endl;
            getchar();
    #endif

            reinitializeOdometryGaussians(r1,r2,tr);

            r1h = r1 - (*_odomModel.rotModel.gaussian)(*_rngen);
            trh = tr - (*_odomModel.transModel.gaussian)(*_rngen);
            r2h = r2 - (*_odomModel.rotModel.gaussian)(*_rngen);

    #ifdef DEBUG
            std::cout << "r1h = " << r1h << " trh = " << trh << " r2h = " << r2h << std::endl;
            getchar();
    #endif

            _Xbar[i].x = _lastX[i].x + trh*cos(_lastX[i].theta + r1h);
            _Xbar[i].y = _lastX[i].y + trh*sin(_lastX[i].theta + r1h);
            _Xbar[i].theta = _lastX[i].theta + r1h + r2h;

    #ifdef DEBUG
            std::cout << "Xx = " << _Xbar[i].x << " Xy = " << _Xbar[i].y << " Xth = " << _Xbar[i].theta << std::endl;
            getchar();
    #endif
        }
    }
}

void ParticleFilter::resampleParticles() {
    std::vector<double> cweights;
    cweights.push_back(_Xbar[0].weight);
    for(int i = 1; i < NUM_PARTICLES; i++ ) {
        cweights.push_back(cweights[i-1] + _Xbar[i].weight);
    }
    int ind;
    for(int i = 0; i < NUM_PARTICLES; i++ ) {
        ind = generateResampledParticle(&cweights);
        std::cout << "ind = " << ind << std::endl;

        _X[i] = _Xbar[ind];
    }

}

int ParticleFilter::generateResampledParticle(std::vector<double>* c) {
    std::uniform_real_distribution<> distro(0.0,(*c)[NUM_PARTICLES-1]);

    double samp = distro(*_rngen);
    std::cout << "samp = " << samp << std::endl;

    return binarySearch(c,samp);
}

int ParticleFilter::binarySearch(std::vector<double>* c, double x) {

    if( x < (*c)[0] ) {
        return 0;
    }
    if( x > (*c)[NUM_PARTICLES-1] ) {
        return NUM_PARTICLES-1;
    }

    int l = 0;
    int h = NUM_PARTICLES-1;
    int mid;

    while ( l <= h ) {
        mid = (h+l)/2;

        if( x < (*c)[mid] ) {
            h = mid-1;
        } else if( x > (*c)[mid] ) {
            l = mid + 1;
        } else {
            return mid;
        }
    }
    return ((*c)[l] - x) < (x - (*c)[h]) ? l : h;
}

void ParticleFilter::plotRawTrajectory() {
    double dx = 0;
    double dy = 0;
    double dth = 0;

    double r1, r2, tr, r1h, r2h, trh;

    std::vector<double> X,Y,Th,Xbar,Ybar,Thbar;
    int c = 0;
    X.push_back(0.0);
    Y.push_back(0.0);
    Th.push_back(0.0);
    Xbar.push_back(0.0);
    Ybar.push_back(0.0);
    Thbar.push_back(0.0);
    //Run over entire data set
    while( _dataReader->increment(&_data) > 0 ) {
        if( _data.type == laserScanData ) {

            if( _firstMeasurement ) {
                _lastData = _data;
                _firstMeasurement = false;
            } else {
                dx = _data.laserData.robotX - _lastData.laserData.robotX;
                dy = _data.laserData.robotY - _lastData.laserData.robotY;
                dth = _data.laserData.robotTheta - _lastData.laserData.robotTheta;
                std::cout << "dx = " << dx << " dy = " << dy << " dth = " << dth << std::endl;

                X.push_back(X[c]   + dx);
                Y.push_back(Y[c]   + dy);
                Th.push_back(Th[c] + dth);
                c++;
                std::cout << "X = " << X[c] << " Y = " << Y[c] << " Th = " << Th[c] << std::endl;

                r1 = atan2(Y[c]-Y[c-1],X[c]-X[c-1]) - Th[c-1];
                dx = X[c-1] - X[c];
                dy = Y[c-1] - Y[c];
                tr = sqrt(dx*dx + dy*dy);
                r2 = Th[c] - Th[c-1] - r1;

                Xbar.push_back(Xbar[c-1] + tr*cos(Thbar[c-1] + r1));
                Ybar.push_back(Ybar[c-1] + tr*sin(Thbar[c-1] + r1));
                Thbar.push_back(Thbar[c-1] + r1 + r2);

                // Save last data frame for next update
                _lastData = _data;
            }
        }
    }
    std::cout << "Plotting.." << std::endl;
    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::scatter(Ybar,Xbar);
    matplotlibcpp::show();
    getchar();
}

void ParticleFilter::plotNoiseTrajectory() {
    double dx = 0;
    double dy = 0;
    double dth = 0;

    double r1, r2, tr, r1h, r2h, trh;

    std::vector<double> X,Y,Th,Xbar,Ybar,Thbar;
    int c = 0;
    X.push_back(0.0);
    Y.push_back(0.0);
    Th.push_back(0.0);
    Xbar.push_back(0.0);
    Ybar.push_back(0.0);
    Thbar.push_back(0.0);
    //Run over entire data set
    while( _dataReader->increment(&_data) > 0 ) {
        if( _data.type == laserScanData ) {

            if( _firstMeasurement ) {
                _lastData = _data;
                _firstMeasurement = false;
            } else {
                dx = _data.laserData.robotX - _lastData.laserData.robotX;
                dy = _data.laserData.robotY - _lastData.laserData.robotY;
                dth = _data.laserData.robotTheta - _lastData.laserData.robotTheta;
                std::cout << "dx = " << dx << " dy = " << dy << " dth = " << dth << std::endl;

                X.push_back(X[c]   + dx);
                Y.push_back(Y[c]   + dy);
                Th.push_back(Th[c] + dth);
                c++;
                std::cout << "X = " << X[c] << " Y = " << Y[c] << " Th = " << Th[c] << std::endl;

                r1 = atan2(Y[c]-Y[c-1],X[c]-X[c-1]) - Th[c-1];
                dx = X[c-1] - X[c];
                dy = Y[c-1] - Y[c];
                tr = sqrt(dx*dx + dy*dy);
                r2 = Th[c] - Th[c-1] - r1;

                reinitializeOdometryGaussians(r1,r2,tr);

                r1h = r1 - (*_odomModel.rotModel.gaussian)(*_rngen);
                trh = tr - (*_odomModel.transModel.gaussian)(*_rngen);
                r2h = r2 - (*_odomModel.rotModel.gaussian)(*_rngen);
                std::cout << "r1g = " << r1h << " trh = " << trh << " r2h = " << r2h << std::endl;

                Xbar.push_back(Xbar[c-1] + trh*cos(Thbar[c-1] + r1h));
                Ybar.push_back(Ybar[c-1] + trh*sin(Thbar[c-1] + r1h));
                Thbar.push_back(Thbar[c-1] + r1h + r2h);

                // Save last data frame for next update
                _lastData = _data;
            }
        }
    }
    std::cout << "Plotting.." << std::endl;
    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::scatter(Ybar,Xbar);
    matplotlibcpp::show();
    getchar();
}

void ParticleFilter::reinitializeOdometryGaussians(double r1, double r2, double tr) {
    _odomModel.rotModel.gaussian = new std::normal_distribution<double>(0,_odomModel.a1*r1*r1 + _odomModel.a2*r2*r2);
    _odomModel.transModel.gaussian = new std::normal_distribution<double>(0,_odomModel.a3*tr*tr + _odomModel.a4*r1*r1 + _odomModel.a4*r2*r2);
}

void ParticleFilter::findImportanceWeights() {
    // Iterate over all particles
    for( int p = 0; p < NUM_PARTICLES; p++ ) {
        // Initialize firs theta as 90 degrees to the left
        double th = -90;

        _Xbar[p].weight = 1;

        // Iterate over all ranges
        for( int i = 0; i < NUM_MEAS; i+=5 ) {
            // Run ray tracer to find expected value from map
            _laserModel.hit.mean = traceMap(_Xbar[p],th)*METER_TO_CM;

            // Calculate weight
            _Xbar[p].weight *= laserModelProb(&_data.laserData.ranges[i]);
#ifdef DEBUG
            std::cout << "_Xbar.weight = " << _Xbar[p].weight << std::endl;
#endif

            // Inrement to new theta (skipping some measurements)
            th += 5;
        }
    }
}

double ParticleFilter::traceMap(particle_t p, double angle) {
    double endX = 0;
    double endY = 0;
    double d = 0;
    bool wallFound = false;
#ifdef DEBUG
    std::cout << "angle = " << angle << std::endl;
#endif
    // Run until wall or max range found
    while( !wallFound && d*METER_TO_CM < _laserModel.max ) {
        // Increment distance to search over
        d += D_DIST;
#ifdef DEBUG
        std::cout << "d = " << d << std::endl;
#endif
        // Calculate new position (in meters)
        endY = p.x + d*sin((p.theta+angle*TO_RADIANS));
        endX = p.y + d*cos((p.theta+angle*TO_RADIANS));
        //std::cout << "START(" << p.y << "," << p.x <<")" << std::endl;
        //std::cout << "END(" << endX << "," << endY <<")" << std::endl;
        _visualizer->flagPixel(endX,endY);
        // Check if pixel is occupied in map
        if( spaceOccupied(endX,endY) ) {
            wallFound = true;
            // // Cycle back until transition found
            // while( spaceOccupied(endX, endY)) {
            //     d -= 0.01;
            //     endX = p.y + d*cos((p.theta+angle*TO_RADIANS));
            //     endY = p.x + d*sin((p.theta+angle*TO_RADIANS));
            //     _visualizer->flagPixel(endX,endY);
            // }
            return d;
        }
    }
}

bool ParticleFilter::spaceOccupied(int x, int y) {
    // Check if space is free
    // std::cout << "Checking " << "X(" << x << ") Y(" << y << ")" << std::endl;
    x = x/PIXEL_RESOLUTION;
    y = y/PIXEL_RESOLUTION;
    if( (MAP_HEIGHT*y + x) < MAP_HEIGHT*MAP_WIDTH ) {
        if( _logicMap[MAP_HEIGHT*y + x] >= 0.5 ) {
            return false;
        }
    }
    return true;
}

uint16_t ParticleFilter::generateEmptyParticleSample() {
    double p = (*_distUni)(*_rngen);
    return static_cast<uint16_t>(p*(NUM_FREE_PARTICLES-1));
}

double ParticleFilter::generateLaserSample() {
    double p = (*_distUni)(*_rngen);
    return static_cast<double>(p*_laserModel.max);
}

double ParticleFilter::generateThetaSample() {
    double p = (*_distUni)(*_rngen);
    return static_cast<double>(p*PI2);
}

void ParticleFilter::calculateLaserNomralizers(double* zstar) {
    _laserModel.nhit  = 1/integrateGaussian(zstar);
    _laserModel.nshrt = 1/(1-exp(-_laserModel.shrt.lambda*_laserModel.hit.mean));
}

double ParticleFilter::integrateGaussian(double* zstar) {
    static double h = _laserModel.max/NUM_BINS;
    static double norm = 1/sqrt(PI2*pow(_laserModel.hit.std,2));
    static double nexp = -0.5/pow(_laserModel.hit.std,2);
    double sum;

    for (int i = 0; i <= NUM_BINS; i++) {
        _x[i]=i*h;            //and store them in arrays
        _y[i]=norm * exp(nexp*pow(_x[i]-(*zstar),2));
        if( i > 1 && i < NUM_BINS ) {
            sum = sum + h*_y[i];
        }
    }
    return _laserModel.max/(2*NUM_BINS)*(_y[0] + sum + _y[NUM_BINS]);
}

double ParticleFilter::laserModelProb(double* zmeas) {
    double p, temp;

    // Handle hit probability
    if( (*zmeas) >= 0 && (*zmeas) <= _laserModel.max ) {
        temp = evaluateGaussian(zmeas);
#ifdef DEBUG
        std::cout << "phit = " << _laserModel.hitFactor*temp;
#endif
        p += _laserModel.hitFactor*temp;
    }
    // Handle short probability
    if( (*zmeas) >= 0 && (*zmeas) <= _laserModel.hit.mean ) {
        temp = exp(-_laserModel.shrt.lambda*(*zmeas));
#ifdef DEBUG
        std::cout << " pshrt = " << _laserModel.shrtFactor*temp;
#endif
        p += _laserModel.shrtFactor*temp;
    }

    // Handle max range probability
    if( (*zmeas) == _laserModel.max ) {
#ifdef DEBUG
        std::cout << " pmax = " << _laserModel.maxFactor;
#endif
        p += _laserModel.maxFactor;
    }

    // Handle random measurement probability
    if( (*zmeas) >= 0 && (*zmeas) < _laserModel.max ) {
#ifdef DEBUG
        std::cout << " prand = " << _laserModel.randFactor*_laserModel.rand;
#endif
        p += _laserModel.randFactor*_laserModel.rand;
    }
#ifdef DEBUG
    std::cout << " ptotal = " << p << std::endl;
#endif
    return p;
}

double ParticleFilter::evaluateGaussian(double* eval) {
    return (1/(sqrt(PI2*pow(_laserModel.hit.std,2))))*exp(-0.5*pow(((*eval)-_laserModel.hit.mean),2))/pow(_laserModel.hit.std,2);
}

void ParticleFilter::visualizeLaserModel() {
    std::vector<double> z(1000);
    std::vector<double> p(1000);

    calculateLaserNomralizers(&_laserModel.hit.mean);
    for( int i = 0; i < 1000; i++ ) {
        z.at(i) = ((double)i/1000)*_laserModel.max;
        p.at(i) = laserModelProb(&z.at(i));
    }

    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::plot(z, p);

    // Set x-axis to interval [0,1000000]
    //matplotlibcpp::xlim(0.0, _laserModel.max);
    matplotlibcpp::show();
    getchar();
}
