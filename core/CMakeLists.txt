cmake_minimum_required(VERSION 3.10)

project(read_t265)

set(USE_OPENCV OFF) # 默认不使用cv库
# 1 、 ON 、 YES 、 true 、 Y 或非零数，则逻辑变量为 true
# 0 、 OFF 、 NO 、 false 、 N 、 IGNORE、 NOTFOUND 、空字符串，或者以 -NOTFOUND 为后缀，则逻辑变量为 false。

set(CMAKE_CXX_STANDARD 11)

find_package(realsense2 REQUIRED)
if(USE_OPENCV)
    find_package(OpenCV REQUIRED)
endif()
include_directories(${realsense2_INCLUDE_DIR})
if(USE_OPENCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()
add_executable(read_t265 read_t265.cpp)

target_link_libraries(read_t265 ${realsense2_LIBRARY})
if(USE_OPENCV)
    target_link_libraries(read_t265 ${OpenCV_LIBS})
endif()
target_link_libraries(read_t265 wiringPi)