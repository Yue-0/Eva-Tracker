cmake_minimum_required(VERSION 3.0.2)
project(simulator)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
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

# Declare C++ libraries
add_library(nav
    src/planner/nav.cpp
)
add_library(simulator 
    src/simulator/env.cpp
    src/simulator/map.cpp
    src/simulator/lidar.cpp
    src/simulator/robot.cpp
)

# Declare C++ executables
add_executable(fov src/fov.cpp)
add_executable(sot src/sot.cpp)
add_executable(env src/simulator.cpp)
add_executable(pose src/transform.cpp)
add_executable(trajectory src/trajectory.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(fov
    ${catkin_LIBRARIES}
)
target_link_libraries(sot
    ${catkin_LIBRARIES}
)
target_link_libraries(env
    nav
    simulator
    ${catkin_LIBRARIES}
)
target_link_libraries(pose
    ${catkin_LIBRARIES}
)
target_link_libraries(trajectory
    ${catkin_LIBRARIES}
)
