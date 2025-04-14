# Use an official Python runtime as a parent image
FROM ubuntu:22.04

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the working directory to /root
ENV DIRPATH=/root
WORKDIR $DIRPATH

#Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y git

RUN apt-get install -y tzdata

# Installing cross-compiling dependencies
RUN apt update -y && apt upgrade -y

RUN apt install -y wget
RUN apt install -y gcc-10 g++-10
RUN apt install -y make
RUN apt install -y cmake
RUN apt install -y build-essential
RUN apt install -y gcc-10-aarch64-linux-gnu
RUN apt install -y g++-10-aarch64-linux-gnu
RUN apt install -y binutils-aarch64-linux-gnu

# my additions
# --- for CLion Integration ---
RUN apt install -y gdb-multiarch
# --- for CLion Integration ---

# Cross Compiling OpenCV (to /usr/local - this path is already in the dynamic linker by default
RUN git clone --filter=blob:none https://github.com/opencv/opencv.git
RUN cd opencv && git checkout tags/4.6.0
RUN git clone --filter=blob:none  https://github.com/opencv/opencv_contrib.git
RUN cd opencv_contrib/ &&  git checkout tags/4.6.0
RUN cd opencv/platforms/linux/ && mkdir build && cd build && \
    cmake -DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc-10 -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++-10\
          -DCMAKE_TOOLCHAIN_FILE=../aarch64-gnu.toolchain.cmake  \
          -DCMAKE_BUILD_TYPE=Release -D BUILD_opencv_python=OFF -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF  \
          -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules  \
          -DCMAKE_INSTALL_PREFIX=/usr/local  \
          ../../.. && \
    make install -j$(nproc)

# Cross-compiling and install arrow (rerun dep)
COPY toolchain.cmake $DIRPATH
RUN git clone --depth 1 --branch apache-arrow-10.0.1 https://github.com/apache/arrow.git
RUN cd arrow/cpp && \
    mkdir build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake  \
          -DCMAKE_BUILD_TYPE=Debug -DARROW_BUILD_STATIC=ON  -DARROW_DEPENDENCY_SOURCE=BUNDLED  \
          -DARROW_ENABLE_THREADING=OFF -DARROW_FLIGHT=OFF -DARROW_JEMALLOC=OFF -DARROW_MIMALLOC=OFF -DARROW_BUILD_SHARED=OFF  \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          ..  &&\
    make install -j$(nproc)


# remove opencv & opencv_contrib directories?
