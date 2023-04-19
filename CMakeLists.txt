cmake_minimum_required(VERSION 3.5)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 20)

include_directories(deps/Eigen)

# include header files
include_directories(include)

# set source files
set(SOURCE_FILES src/main.cpp src/KalmanFilter.cpp)

# generate executable
add_executable(KalmanFilter ${SOURCE_FILES})


