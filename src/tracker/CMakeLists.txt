cmake_minimum_required(VERSION 3.0.2)
project(tracker)

set(REAL_WORLD TRUE)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    message_generation
    pcl_conversions
    quadrotor_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    roscpp
    tf
)

# System dependencies are found with CMake's conventions
find_package(OpenMP REQUIRED)
if(${REAL_WORLD})
    find_package(CUDA REQUIRED)
    find_package(OpenCV REQUIRED)
endif()

# Generate message
if(${REAL_WORLD})
    add_message_files(
        FILES
        Camera.msg
    )
    generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
    )
endif()

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
if(${REAL_WORLD})
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/cuda/include
        /usr/include/aarch64-linux-gnu
    )
else()
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )
endif()

# Declare C++ libraries
add_library(bezier
    src/predictor/bezier.cpp
)
add_library(eva_tracker
    src/planner/map.cpp
    src/planner/esdf.cpp
    src/planner/plan.cpp
    src/planner/minco.cpp
    src/planner/optimizer.cpp
)
add_library(predictor
    src/predictor/predictor.cpp
)
if(${REAL_WORLD})
    add_library(detection
        src/perception/yolo.cpp
        src/perception/tracker.cpp
        src/perception/monoloco.cpp
    )
endif()
target_compile_options(eva_tracker PRIVATE ${OpenMP_CXX_FLAGS})

# Declare a C++ executables
add_executable(tracking src/planner.cpp)
add_executable(prediction src/prediction.cpp)
if(${REAL_WORLD})
    add_executable(perception src/perception.cpp)
endif()

# Specify libraries to link a library or executable target against
target_link_libraries(tracking PRIVATE ${OpenMP_CXX_LIBRARIES}
    bezier
    eva_tracker
    ${catkin_LIBRARIES}
)
target_link_libraries(prediction
    bezier
    predictor
    ${catkin_LIBRARIES}
)
if(${REAL_WORLD})
    target_link_libraries(perception
        bezier
        detection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}

        # For CUDA inference
        cudart
        nvinfer
        libnvinfer_plugin.so
    )
endif()

# Mark executable python scripts
if(${REAL_WORLD})
    catkin_install_python(PROGRAMS
        scripts/camera.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    catkin_install_python(PROGRAMS
        scripts/control.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
