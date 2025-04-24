cmake_minimum_required(VERSION 3.16.3)
project(IMX)

set(CMAKE_CXX_STANDARD 14)
include_directories(/usr/local/include) #just in case?


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)
#set(GTSAM_LIBRARIES /usr/local/lib/libgtsam.so)
include_directories(${GTSAM_INCLUDE_DIR})

# Download the rerun_sdk (ver 0.22); may have to explicitly download ver 0.22 in future; future rerun updates may include brea>
include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


add_executable(IMX tiny_arm_slam.cpp)

target_link_libraries(IMX PRIVATE ${OpenCV_LIBS} ${GTSAM_LIBRARIES} gtsam rerun_sdk)