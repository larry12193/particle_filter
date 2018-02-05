cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)
add_compile_options(-std=c++11)

find_package( Boost REQUIRED COMPONENTS program_options )

include_directories(include
                    ${Boost_INCLUDE_DIRS})

add_library(logReader src/logReader.cpp)

# Define sources and executable
set(EXECUTABLE_NAME "particle_filter")
add_executable(${EXECUTABLE_NAME} src/main.cpp)

target_link_libraries(${EXECUTABLE_NAME}
                      logReader
                      ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

# Detect and add SFML
#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
#Find any version 2.X of SFML
#See the FindSFML.cmake file for additional details and instructions
#find_package(SFML 2 REQUIRED network audio graphics window system)
#if(SFML_FOUND)
#  include_directories(${SFML_INCLUDE_DIR})
#  target_link_libraries(${EXECUTABLE_NAME} ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
#endif()

# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
