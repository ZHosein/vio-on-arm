cmake_minimum_required(VERSION 3.22.1)
project(Dev)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions("SOURCE_ROOT=${CMAKE_SOURCE_DIR}")


# Linking boost (for GTSAM)
find_package(Boost 1.65.0 REQUIRED system filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
# Adding path to Eigen to includes (for GTSAM)
find_package(GTSAM REQUIRED)
include_directories(/usr/local/include/gtsam/3rdparty/)
include_directories(/usr/local/include/gtsam/3rdparty/Eigen)


#set(OpenCV_DIR /home/<username>/opencv/lib/cmake/opencv4) # leave commented out if OpenCV is installed
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})


# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        #https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip) # for the latest version of rerun
        https://github.com/rerun-io/rerun/releases/download/0.17.0/rerun_cpp_sdk.zip) # for rerun 0.17
FetchContent_MakeAvailable(rerun_sdk)



add_executable(Dev main.cpp)
#add_executable(gtsamExample GTSAMVisualISAMExample.cpp )
add_executable(gtsamExample "GTSAM Examples/factorGraphs.cpp" )

target_link_libraries(Dev ${OpenCV_LIBS} rerun_sdk ${Boost_LIBRARIES} metis-gtsam gtsam gtsam_unstable)
target_link_libraries(gtsamExample ${Boost_LIBRARIES} metis-gtsam gtsam gtsam_unstable)
