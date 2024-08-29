cmake_minimum_required(VERSION 3.0.2)
project(tracker)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    geometry_msgs
    sensor_msgs
    nav_msgs
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

# Declare a C++ library
add_library(eva_tracker
    src/searcher/map.cpp
    src/searcher/search.cpp
    src/optimizer/esdf.cpp
    src/optimizer/optim.cpp
    src/optimizer/bspline.cpp
)

# Declare a C++ executable
add_executable(tracking src/track.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(tracking
    eva_tracker
    ${catkin_LIBRARIES}
)
