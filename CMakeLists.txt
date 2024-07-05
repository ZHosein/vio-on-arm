cmake_minimum_required(VERSION 3.22.1)
project(IMX)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(IMX main.cpp)

target_link_libraries(IMX PRIVATE ${OpenCV_LIBS})
