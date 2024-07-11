cmake_minimum_required(VERSION 3.22.1)
project(IMX)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(IMX main.cpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


target_link_libraries(IMX PRIVATE ${OpenCV_LIBS} rerun_sdk)
