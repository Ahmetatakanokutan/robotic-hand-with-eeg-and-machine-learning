cmake_minimum_required(VERSION 3.10)
project(EyeTrackingRobot)

set(CMAKE_CXX_STANDARD 14)

# FIND OPENCV
# You typically need to set OpenCV_DIR environment variable or hardcode it
# e.g. set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(EyeTrackingRobot main.cpp)

target_link_libraries(EyeTrackingRobot ${OpenCV_LIBS})

# Instructions:
# 1. Install CMake
# 2. Install OpenCV (e.g. C:\opencv)
# 3. mkdir build && cd build
# 4. cmake -DOpenCV_DIR=C:\opencv\build ..
# 5. cmake --build . --config Release
