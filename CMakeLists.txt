cmake_minimum_required(VERSION 3.10)
project(QuadrupedObserver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
endif()

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
    ${EIGEN3_INCLUDE_DIR}
)

# Compile generated dynamics code
add_library(quadruped_dynamics
    cpp/quadruped_dynamics.cpp
    cpp/DisturbanceObserver.cpp
)
target_link_libraries(quadruped_dynamics Eigen3::Eigen)

# Main program
add_executable(quadruped_observer
    cpp/main.cpp
)
target_link_libraries(quadruped_observer
    quadruped_dynamics
    Eigen3::Eigen
)

# Installation rules
install(TARGETS quadruped_observer DESTINATION bin)

# Unit tests (optional)
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()