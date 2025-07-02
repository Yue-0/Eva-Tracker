cmake_minimum_required(VERSION 3.0.2)
project(simulator)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    quadrotor_msgs
    geometry_msgs
    sensor_msgs
    roscpp
    tf
)

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Declare C++ library
add_library(simulator
    src/simulator/map.cpp
    src/simulator/lidar.cpp
    src/simulator/planner.cpp
    src/simulator/tracker.cpp
)

# Declare C++ executable
add_executable(env src/simulator.cpp)

# Specify library to link a library or executable target against
target_link_libraries(env
    simulator
    ${catkin_LIBRARIES}
)
