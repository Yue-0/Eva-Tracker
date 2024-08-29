cmake_minimum_required(VERSION 3.0.2)
project(predictor)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
)

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
include_directories(
    include
    ../tracker/include
    ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(predictor
    src/predictor/predictor.cpp
)

# Declare a C++ executable
add_executable(prediction src/prediction.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(prediction
    predictor
    ${catkin_LIBRARIES}
)
