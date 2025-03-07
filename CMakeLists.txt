cmake_minimum_required(VERSION 3.22.1)
project(IMX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
set(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc-8")
set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++-8")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(opencv/modules/core/include)

add_executable(IMX main.cpp)

# rerun installed from script previously installed
# find_package(rerun_sdk REQUIRED)
# Download the rerun_sdk
include(FetchContent)

# set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchain.cmake")
# set(CMAKE_BUILD_TYPE Release)
# set(ARROW_BUILD_STATIC ON)
# set(ARROW_DEPENDENCY_SOURCE BUNDLED)
# set(ARROW_ENABLE_THREADING OFF)
# set(ARROW_FLIGHT OFF)
# set(ARROW_JEMALLOC OFF)
# set(ARROW_MIMALLOC OFF)
# set(ARROW_BUILD_SHARED OFF)
# set(CMAKE_INSTALL_PREFIX /usr/local)

FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


target_link_libraries(IMX PRIVATE ${OpenCV_LIBS} rerun_sdk)
target_link_options(IMX PRIVATE -static -static-libgcc -static-libstdc++)
