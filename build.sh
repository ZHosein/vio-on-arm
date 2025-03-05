# docker build -t vio-on-arm-imx
apt-get update -y && apt-get install -y --no-install-recommends apt-utils && apt upgrade -y
apt-get update -y && apt-get install -y git cmake tzdata
apt install -y wget gcc make build-essential gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    binutils-aarch64-linux-gnu gdb-multiarch

# Cross Compiling OpenCV (to /usr/local - this path is already in the dynamic linker by default
if [ ! -d opencv_contrib ]; then
    git clone --branch 4.6.0 --depth=1 https://github.com/opencv/opencv_contrib.git
fi
if [ ! -d opencv ]; then
    git clone --branch 4.6.0 --depth=1 https://github.com/opencv/opencv.git
    cd opencv/platforms/linux/ && mkdir build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=../aarch64-gnu.toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release -D BUILD_opencv_python=OFF -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF \
        -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ../../.. && \
    make -j $(nproc) && \
    make install
    cd ../../..
fi

if [ ! -d arrow ]; then
    git clone --depth=1 https://github.com/apache/arrow.git
    cd arrow/cpp && \
    mkdir "build" && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Debug -DARROW_BUILD_STATIC=ON -DARROW_DEPENDENCY_SOURCE=BUNDLED \
        -DARROW_ENABLE_THREADING=OFF -DARROW_FLIGHT=OFF -DARROW_JEMALLOC=OFF -DARROW_MIMALLOC=OFF -DARROW_BUILD_SHARED=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j $(nproc) && make install
    cd ../..
fi