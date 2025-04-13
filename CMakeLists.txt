cmake_minimum_required(VERSION 3.16.3)
project(IMX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc-8")
set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++-8")
include_directories(/usr/local/include) #just in case?

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(opencv/modules/core/include)

add_executable(IMX main.cpp)

include_directories(/usr/local/boost-arm/include)
find_package(GTSAM REQUIRED)
set(GTSAM_LIBRARIES /usr/local/lib/libgtsam.so)
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen)

# rerun installed from script previously installed
# find_package(rerun_sdk REQUIRED)
# Download the rerun_sdk
include(FetchContent)

FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


target_link_libraries(IMX PRIVATE ${OpenCV_LIBS} ${GTSAM_LIBRARIES} rerun_sdk)
