cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)
add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

find_package( Boost REQUIRED COMPONENTS thread system program_options filesystem)

include_directories(include
                    ${Boost_INCLUDE_DIRS})

add_library(particleFilter src/visualizer.cpp src/logReader.cpp src/particleFilter.cpp)

# Define sources and executable
set(EXECUTABLE_NAME "particle_filter")
add_executable(${EXECUTABLE_NAME} src/main.cpp)

target_link_libraries(${EXECUTABLE_NAME}
                      ${Boost_LIBRARIES}
                      particleFilter
                      sfml-graphics
                      sfml-window
                      sfml-system
                      pthread
                      boost_thread)

# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
