cmake_minimum_required(VERSION 3.5)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 20)

include_directories(deps/Eigen)

# include header files
include_directories(include)

# set source files
set(SOURCE_FILES src/main.cpp src/KalmanFilter.cpp src/gps_data.cpp)

# generate executable
add_executable(KalmanFilter ${SOURCE_FILES})

# include nlohmann_json
find_package(nlohmann_json 3.2.0 REQUIRED)
target_include_directories(${PROJECT_NAME} PRIVATE ${nlohmann_json_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)


