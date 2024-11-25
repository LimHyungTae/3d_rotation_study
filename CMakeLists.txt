cmake_minimum_required(VERSION 3.10)
project(rotation_study)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable target
add_executable(main main.cpp)

# Link libraries
target_link_libraries(main gtsam Eigen3::Eigen ${OpenCV_LIBS})

