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

    memset(&_X,0,sizeof(_X));
    memset(&_Xbar,0,sizeof(_Xbar));

    // Generate inital particle set
    this->generateParticles();
    // _X[0].x = 412;
    // _X[0].y = 400;
    // _X[0].theta = -PI;
    // memcpy(_Xbar,_X,sizeof(_X));
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
    // visualizeLaserModel();
    int step = 0;
    //Run over entire data set
    while( _dataReader->increment(&_data) > 0 ) {
        step++;
        if( _firstMeasurement ) {
            printf("Saving first data\n");
            _lastData = _data;
            _firstMeasurement = false;
        } else if( _data.type == laserScanData ) {

            // findImportanceWeights();
            // Propogate points using odometry
            applyMotionModel();
            // printf("Done applying motion model\n");
            // Find importance weights based on sensor model
            findImportanceWeights();
            // printf("Done finding weights\n");
            // for( int i = 0; i < NUM_PARTICLES; i++ ) {
            //     printf("Xbar[%d].weight = %f\n",i,_Xbar[i].weight);
            //     // std::cout << "Xbar" << i <<  " weight = " << pow(10,_Xbar[i].weight) << std::endl;
            // }
            // getchar();
            // Resample particles
            lowVarResampleParticles();
            // printf("Done resampling particles");
            // memcpy(_X,_Xbar,sizeof(_X));
            _visualizer->drawParticles(&_X[0]);
            printf("Step #%d at t = %f\n",step,_data.laserData.time);
        } else {
            // applyMotionModel();
            // lowVarResampleParticles();
            // // printf("Done resampling particles");
            // // memcpy(_X,_Xbar,sizeof(_X));
            // _visualizer->drawParticles(&_X[0]);

            // Save last data frame for next update
        }
        _lastData = _data;
        usleep(50000);
        getchar();
    }
    std::cout << "Particle filter complete" << std::endl;

    // //Run over entire data set
    // while( _dataReader->increment(&_data) > 0 ) {
    //     if( _data.type == laserScanData ) {
    //
    //         if( _firstMeasurement ) {
    //             printf("Saving first data\n");
    //             _lastData = _data;
    //             _firstMeasurement = false;
    //         } else {
    //
    //             // findImportanceWeights();
    //             // Propogate points using odometry
    //             applyMotionModel();
    //             // printf("Done applying motion model\n");
    //             // Find importance weights based on sensor model
    //             findImportanceWeights();
    //             // printf("Done finding weights\n");
    //             // for( int i = 0; i < NUM_PARTICLES; i++ ) {
    //             //     printf("Xbar[%d].weight = %f\n",i,pow(10.0,_Xbar[i].weight));
    //             //     // std::cout << "Xbar" << i <<  " weight = " << pow(10,_Xbar[i].weight) << std::endl;
    //             // }
    //             // Resample particles
    //             lowVarResampleParticles();
    //             // printf("Done resampling particles");
    //             // memcpy(_X,_Xbar,sizeof(_X));
    //             _visualizer->drawParticles(&_X[0]);
    //
    //             // Save last data frame for next update
    //             _lastData = _data;
    //         }
    //         usleep(500000);
    //         //getchar();
    //     }
    // }
    // std::cout << "Particle filter complete" << std::endl;
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
        _X[i].y = (MAP_HEIGHT - std::atof(token.c_str()));

        // Remove data from string
        line.erase(0,pos+delimeter.size());

        // Load data into current memory location
        _X[i].x = std::atof(line.c_str());

        // Generate theta estimate
        _X[i].theta = -PI;//generateThetaSample();

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
    // printf("a1 = %f\n",_odomModel.a1);

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a2 = std::atof(token.c_str());
    // printf("a2 = %f\n",_odomModel.a2);

    // Extract rotational model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a3 = std::atof(token.c_str());
    // printf("a3 = %f\n",_odomModel.a3);

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _odomModel.a4 = std::atof(token.c_str());
    // printf("a4 = %f\n",_odomModel.a4);

    // Extract laser hit model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.hit.mean = std::atof(token.c_str());
    // printf("hit mean = %f\n",_laserModel.hit.mean);

    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.hit.std = std::atof(token.c_str());
    // printf("hit std = %f\n",_laserModel.hit.std);

    // Initialize rotational gaussian
    _laserModel.hit.gaussian = new std::normal_distribution<double>(_laserModel.hit.mean,_laserModel.hit.std);

    // Extract laser unexpected objects model parameters
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.shrt.lambda = std::atof(token.c_str());
    _laserModel.shrt.exp = new std::exponential_distribution<double>(_laserModel.shrt.lambda);
    // printf("shrt lambda = %f\n",_laserModel.shrt.lambda);

    // Extract laser max range
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.max = std::atof(token.c_str());
    // printf("max = %f\n",_laserModel.max);

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
    // printf("hit fac = %f\n",_laserModel.hitFactor);

    // Extract laser short factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.shrtFactor = std::atof(token.c_str());
    // printf("sgrt fac = %f\n",_laserModel.shrtFactor);

    // Extract laser max factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.maxFactor = std::atof(token.c_str());
    // printf("max fac = %f\n",_laserModel.maxFactor);

    // Extract laser rand factor
    getline(_openFD, line);
    pos = line.find(delimeter);
    token = line.substr(0,pos);
    _laserModel.randFactor = std::atof(token.c_str());
    // printf("rand fac = %f\n",_laserModel.randFactor);

    _openFD.close();
}

void ParticleFilter::loadLogicalMap() {

    // std::ofstream outfd;
    // outfd.open("/home/lawrence/Desktop/out.txt", std::ifstream::out);

    std::ifstream fd;
    fd.open(map_data.c_str(),std::ifstream::in);
    size_t pos = 0;
    std::string token, line;
    int row, col;
    row = 0;
    col = 0;
    float data;

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
            data = std::atof(token.c_str());
            _logicMap[row][col] = data;
            col++;
            // Remove data from string
            line.erase(0,pos+delimeter.size());
        }
        row++;
        col = 0;
        // outfd << std::endl;;
        dataValid = true;
    }

    // for( int x = 1; x <= MAP_HEIGHT; x++ ) {
    //     for( int y = 1; y <= MAP_WIDTH; y++ ) {
    //         outfd << _logicMap[MAP_HEIGHT*(x-1) + y - 1] << " ";
    //     }
    //     outfd << std::endl;
    // }
    fd.close();
    // outfd.close();
}

void ParticleFilter::applyMotionModel() {

    double dx, dy, dth;
    double r1, r2, tr, r1h, r2h, trh;
    double s1,s2,s3;

    // Incremental odometry
    dx  = (_data.laserData.robotX    - _lastData.laserData.robotX)/10;
    dy  = (_data.laserData.robotY    - _lastData.laserData.robotY)/10;
    dth = (_data.laserData.robotTheta - _lastData.laserData.robotTheta);
    // Raw components
    r1    = atan2(dy,dx) - _lastData.laserData.robotTheta;
    tr    = sqrt(dx*dx + dy*dy);
    r2    = dth - r1;
    // Initialize gaussians with raw valuess
    reinitializeOdometryGaussians(r1, r2, tr);

    // Iterate over all particles
    for( int i = 0; i < NUM_PARTICLES; i++ ) {
        // Generate samples
        s1 = (*_odomModel.rotModel1.gaussian)(*_rngen);
        s2 = (*_odomModel.transModel.gaussian)(*_rngen);
        s3 = (*_odomModel.rotModel2.gaussian)(*_rngen);
        // Update components
        r1h    = r1    - s1;//(*_odomModel.rotModel1.gaussian)(*_rngen);
        trh    = tr    - s2;// (*_odomModel.transModel.gaussian)(*_rngen);
        r2h    = r2    - s3;//(*_odomModel.rotModel2.gaussian)(*_rngen);
        // Update particles
        _Xbar[i].x     = _X[i].x + trh*cos(_X[i].theta + r1h);
        _Xbar[i].y     = _X[i].y + trh*sin(_X[i].theta + r1h);
        _Xbar[i].theta = _X[i].theta + r1h + r2h;
    }
}

double ParticleFilter::sample(double b) {
    double r;
    _distSamp = new std::uniform_real_distribution<double>(-b,b);
    for( int i = 0; i < 12; i++ ) {
        r += (*_distSamp)(*_rngen);
    }
    return r/12.0;
}

void ParticleFilter::lowVarResampleParticles() {
    // Generate random sample between 0 and M^-1
    double r = (*_distUni)(*_rngen)/NUM_PARTICLES;
    double eta = 0;
    std::vector<int> skipped_particles;

    for( int k = 0; k < NUM_PARTICLES; k++ ) {
        eta += _Xbar[k].weight;
        // eta+= pow(10,_Xbar[k].weight);
    }

    // Initialize running weight
    // double c = pow(10.0,_Xbar[0].weight)/eta;
    double c = _Xbar[0].weight/eta;
    // Initialize weight index
    int i = 0;
    // Combing variable
    double u;

    // Generate M number of new samples
    for( int m = 0; m < (NUM_PARTICLES); m++ ) {
        // Increment to next comb position
        u = r + (float)m/NUM_PARTICLES;
        while( u > c/eta ) {
            // Increment the weighted particle index
            skipped_particles.push_back(i);
            i++;
            // Add next particle weight to running weight
            // c += pow(10.0,_Xbar[i].weight)/eta;
            c += _Xbar[i].weight;
        }
        // Save particle to final set
        _X[m] = _Xbar[i];
    }
    //
    // size_t pos = 0;
    // uint16_t sampleRow;
    // std::string line, token;
    // _openFD.open(open_data.c_str(), std::ifstream::in);
    //
    // // Check to make sure file was successfully opened
    // if( !_openFD.good() ) {
    //     perror("Error opening open spaces data file.");
    // }
    //
    // // Generate particles
    // int len = (skipped_particles.size() > 1000 ? 1000 : skipped_particles.size());
    // printf("s = %d, len = %d", (int)skipped_particles.size(), len);
    // for( int m = 0; m < len; m++ ) {        // Grab row sample
    //     sampleRow = generateEmptyParticleSample();
    //
    //     _openFD.seekg(0, _openFD.beg);
    //
    //     // Get to row
    //     for( int j = 0; j < sampleRow; j++ ) {
    //         getline(_openFD, line);
    //     }
    //
    //     // Find position of delimeter
    //     pos = line.find(delimeter);
    //     // Extract substring bounded by delimeter
    //     token = line.substr(0,pos);
    //     // Load data into current memory location
    //     _X[skipped_particles.at(m)].y = (MAP_HEIGHT - std::atof(token.c_str()));
    //
    //     // Remove data from string
    //     line.erase(0,pos+delimeter.size());
    //
    //     // Load data into current memory location
    //     _X[skipped_particles.at(m)].x = std::atof(line.c_str());
    //
    //     // Generate theta estimate
    //     _X[skipped_particles.at(m)].theta = generateThetaSample();
    // }
    // _openFD.close();
}

void ParticleFilter::reinitializeOdometryGaussians(double r1, double r2, double tr) {
    _odomModel.rotModel1.gaussian  = new std::normal_distribution<double>(0,_odomModel.a1*r1*r1 + _odomModel.a2*tr*tr);
    _odomModel.rotModel2.gaussian  = new std::normal_distribution<double>(0,_odomModel.a1*r2*r2 + _odomModel.a2*tr*tr);
    _odomModel.transModel.gaussian = new std::normal_distribution<double>(0,_odomModel.a3*tr*tr + _odomModel.a4*r1*r1 + _odomModel.a4*r2*r2);
}

void ParticleFilter::findImportanceWeights() {

    double zmap;
    int range = 0;
    std::vector<double> ZMAP, SCAN, TH;
    // Iterate over all particles
    for(int p = 0; p < NUM_PARTICLES; p++ ) {
        if( spaceUnoccupied(_Xbar[p].x, _Xbar[p].y) ) {
            // Initialize particle weight
            _Xbar[p].weight = 1;
            range = 0;
            // Iterate over all ranges
            // _visualizer->drawScan(&_Xbar[p],_data.laserData.ranges);
            for(int th = -90; th < 90; th+=RANGE_ANGLE_DELTA ) {
                // Trace map to find particle's true measurement and update laser model
                zmap = rayTrace(_Xbar[p], th*PI/180.0);

                // ZMAP.push_back(zmap*PIXEL_RESOLUTION);
                // SCAN.push_back(_data.laserData.ranges[range]);
                // TH.push_back(th*TO_RADIANS);

                // printf("Found trace %f",zmap);
                if( zmap > 0 ) {
                    // Update laser model mean value with true measurement
                    _laserModel.hit.mean = zmap*PIXEL_RESOLUTION;
                    // printf("zmap = %f\n",zmap*PIXEL_RESOLUTION);
                    // Add probability of measurement to entire particle weight (via log)
                    double w = (laserModelProb(_data.laserData.ranges[range]));
                    // printf("ws = %f, r = %f\n",w,_data.laserData.ranges[range]);
                    _Xbar[p].weight *= w;
                    // printf("W = %f\n",_Xbar[p].weight);
                } else {
                    _Xbar[p].weight *= 0.001;
                }
                range+=(RANGE_ANGLE_DELTA);
            }
            // // Plot line from given x and y data. Color is selected automatically.
            // matplotlibcpp::polar(TH, ZMAP);
            // matplotlibcpp::polar(TH, SCAN);
            // matplotlibcpp::show();
            // matplotlibcpp::clf();
            // TH.clear();
            // ZMAP.clear();
            // SCAN.clear();
            // getchar();
        } else {
            _Xbar[p].weight = 0.00001;
        }

    }
}

double ParticleFilter::rayTrace(particle_t p, double angle) {
    double endX, endY, lastX, lastY, d, px, py;

    // Intialize the search distance
    d = 0;
    double lx = (LASER_OFFSET_X/PIXEL_RESOLUTION)*cos(p.theta);
    double ly = (LASER_OFFSET_X/PIXEL_RESOLUTION)*sin(p.theta);
    // printf("pth = %f", p.theta);
    px = p.x + lx;
    py = MAP_HEIGHT - (p.y + ly);
    // Calculate the endpoint (in meters) of search line
    endX = px;
    endY = py;

    // printf("ENDX = %f, ENDY = %f\n",endX,endY);
    // Run trace while a wall is not found
    int ret = spaceUnoccupied(endX,endY);
    // printf("ret = %d\n",ret);
    while( ret > 0  && d < _laserModel.max ) {
        d += SEARCH_DIST;
        endX = px + d*cos(p.theta-angle);
        endY = py + d*sin(p.theta-angle);
        ret = spaceUnoccupied(endX,endY);
        // _visualizer->flagPixel(endX,endY);
        // getchar();
    }
    // printf("ret = %d, d = %f",ret,d);
    if( ret == 0 && d != 0 ) {
        return sqrt(pow(endX-px,2) + pow(endY-py,2));
    } else if( ret == 0 && d == 0 ) {
        return -1;
    } else {
        return _laserModel.max;
    }
}

int ParticleFilter::spaceUnoccupied(double x, double y) {

    int px, py;
    // Convert xy position to pixel value
    px = (int)(round(x));
    py = MAP_HEIGHT - (int)(round(y));

    // Make sure target is within range
    if( (px+1) < MAP_WIDTH && (py+1) < MAP_HEIGHT && px > 0 && py > 0 ) {
        // Check if the pixel probability is less than a half (meaning more likely unoccupied)
        if( _logicMap[py][px] > 0.95 ) {
            return 1;
        }
        return 0;
    }
    return -1;
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

void ParticleFilter::calculateLaserNomralizers() {
    _laserModel.nhit  = 1/integrateGaussian(&_laserModel.hit.mean);
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

double ParticleFilter::laserModelProb(double zmeas) {
    double p, temp;
    p = 0;
    temp = 0;
    // Handle hit probability
    if( zmeas >= 0 && zmeas <= _laserModel.max ) {
        temp = evaluateGaussian(zmeas);
        p += _laserModel.hitFactor*temp;
        // printf("phit = %f", _laserModel.hitFactor*temp);
    }
    // Handle short probability
    if( zmeas >= 0 && zmeas <= _laserModel.hit.mean ) {
        temp = _laserModel.shrt.lambda*exp(-_laserModel.shrt.lambda*zmeas);
        p += _laserModel.shrtFactor*temp;
        // printf(" pshrt = %f", _laserModel.shrtFactor*temp);
    }

    // Handle max range probability
    if( zmeas >= 0.99*_laserModel.max ) {
        p += _laserModel.maxFactor;
        // printf(" pmax = %f", _laserModel.maxFactor);
    }

    // Handle random measurement probability
    if( zmeas >= 0 && zmeas < _laserModel.max ) {
        p += _laserModel.randFactor*(1.0/_laserModel.max);
        // printf(" prand = %f\n", _laserModel.randFactor/_laserModel.max);
    }
    // printf("p = %f\n",p);
    // getchar();
    return p;
}

double ParticleFilter::evaluateGaussian(double eval) {
    double var = _laserModel.hit.std*_laserModel.hit.std;
    return (1/(sqrt(PI2*var)))*exp(-0.5*pow((eval-_laserModel.hit.mean),2)/var);
}


void ParticleFilter::plotRawTrajectory() {
    double dx = 0;
    double dy = 0;
    double dth = 0;

    double r1, r2, tr;

    std::vector<double> X,Y,Th,Xbar,Ybar,Thbar;
    int c = 1;
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
                dx  = (_data.laserData.robotX     - _lastData.laserData.robotX)*CM_TO_METERS;
                dy  = -(_data.laserData.robotY     - _lastData.laserData.robotY)*CM_TO_METERS;
                dth = (_data.laserData.robotTheta - _lastData.laserData.robotTheta);

                r1    = atan2(dy,dx) - _lastData.laserData.robotTheta;
                tr    = sqrt(dx*dx + dy*dy);
                r2    = dth - r1;

                Xbar.push_back(Xbar[c-1] + tr*cos(Thbar[c-1] + r1));
                Ybar.push_back(Ybar[c-1] - tr*sin(Thbar[c-1] + r1));
                Thbar.push_back(Thbar[c-1] + r1 + r2);
                c++;
                // Save last data frame for next update
                _lastData = _data;
            }
        }
    }
    std::cout << "Plotting.." << std::endl;
    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::scatter(Xbar,Ybar);
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

                r1h = r1 - (*_odomModel.rotModel1.gaussian)(*_rngen);
                trh = tr - (*_odomModel.transModel.gaussian)(*_rngen);
                r2h = r2 - (*_odomModel.rotModel2.gaussian)(*_rngen);
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

void ParticleFilter::visualizeLaserModel() {
    std::vector<double> z(1000);
    std::vector<double> p(1000);

    calculateLaserNomralizers();
    for( int i = 0; i < 1000; i++ ) {
        z.at(i) = ((double)i/1000)*_laserModel.max;
        p.at(i) = laserModelProb(z.at(i));
    }

    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::plot(z, p);

    // Set x-axis to interval [0,1000000]
    //matplotlibcpp::xlim(0.0, _laserModel.max);
    matplotlibcpp::show();
    getchar();
}
