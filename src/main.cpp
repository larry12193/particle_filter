/** @file main.cpp
 *  @brief Contains the main function which runs the particle filter on a given
 *         data set.
 *
 *  @author Lawrence Papincak (larry12193)
 *  @bug No known bugs.
 */

#include "logReader.h"
#include "particleFilter.h"
#include <boost/program_options.hpp>

#define BOLDBLUE "\033[1m\033[94m"
#define BOLDRED  "\033[1m\033[31m"
#define ENDC   "\033[0m"

namespace pots = boost::program_options;

int main(int argc, char *argv[]) {

    std::string dataPath, outputPath;
    int8_t result;
    dataFrame_t currentData;

    // Parse command-line arguments
    pots::options_description desc("Allowed Options");
    desc.add_options()
        ("help,h",   "Allowed Options:")
        ("data,d",   pots::value<std::string>()->required(), "The path to data file to be processed.")
        ("output,o", pots::value<std::string>()->required(), "The path where output files will be generated.")
        ;

    try {
        pots::variables_map vm;
        pots::store(pots::parse_command_line(argc, argv, desc),vm);
        pots::notify(vm);

        dataPath = vm["data"].as<std::string>();
        outputPath = vm["output"].as<std::string>();

    } catch ( const std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    std::cout << BOLDBLUE << "Processing " << dataPath << ENDC << std::endl;
    std::cout << BOLDBLUE << "Generating output in " << outputPath << ENDC << std::endl;

    // Create instance of particle filter solver
    ParticleFilter pf(dataPath, outputPath);
    pf.solve();
    //pf.generateParticles();
    //pf.plotRawTrajectory();
    // pf.plotNoiseTrajectory();
    return 0;
}
