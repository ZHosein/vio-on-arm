# user required: root (sudo su -)
# docker command: docker build -t vio-on-arm-imx
# shell script command: bash build.sh
export TZ=${TZ:-Etc/UTC}
export DIRPATH=${DIRPATH:-/root}
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=/usr/bin/aarch64-linux-gnu-gcc-8
export CC_aarch64_unknown_linux_gnu=/usr/bin/aarch64-linux-gnu-gcc-8
export CXX_aarch64_unknown_linux_gnu=/usr/bin/aarch64-linux-gnu-g++-8
export CARGO_TARGET=aarch64-unknown-linux-gnu
export AR=/usr/bin/aarch64-linux-gnu-ar
export LD=/usr/bin/aarch64-linux-gnu-ld
export PATH="$HOME/.cargo/bin:$PATH"
export RUSTFLAGS="-C linker=/usr/bin/aarch64-linux-gnu-gcc-8"
#you can place the above lines at the end of your .bashrc file

CURRENT_DIR=$(pwd)
if [ "$CURRENT_DIR" != "$DIRPATH" ]; then
    cp -r ./* ./.?* $DIRPATH/
    cd $DIRPATH
    rm -rf $CURRENT_DIR
fi
sed -i 's/azure\.//g' /etc/apt/sources.list
apt-get update -y && apt-get install -y --no-install-recommends apt-utils && apt upgrade -y
apt-get update -y && apt-get install -y git cmake tzdata
apt install -y curl wget gcc make build-essential gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    binutils-aarch64-linux-gnu gdb-multiarch
add-apt-repository -y ppa:ubuntu-toolchain-r/test
apt-get update -y && apt-get upgrade -y
apt install -y gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu gcc-8 g++-8 --fix-missing
apt --fix-broken -y install
apt install -y gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu gcc-8 g++-8

# apt install -y libgmp-dev libmpfr-dev libmpc-dev
# wget http://ftp.gnu.org/gnu/gcc/gcc-8.5.0/gcc-8.5.0.tar.gz
# tar -xvzf gcc-8.5.0.tar.gz
# cd gcc-8.5.0
# ./contrib/download_prerequisites
# mkdir build
# cd build
# ../configure --target=aarch64-linux-gnu --enable-languages=c,c++ --disable-multilib --prefix=/usr
# make -j$(nproc) install

# Cross Compiling OpenCV (to /usr/local - this path is already in the dynamic linker by default
if [ ! -d opencv_contrib ]; then
    git clone --branch 4.6.0 --depth=1 https://github.com/opencv/opencv_contrib.git
fi
if [ ! -d opencv ]; then
    git clone --branch 4.6.0 --depth=1 https://github.com/opencv/opencv.git
    cd opencv/platforms/linux/ && mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=../aarch64-gnu.toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release -D BUILD_opencv_python=OFF -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF \
        -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ../../.. && \
    make -j $(nproc) && \
    make install
    cd ../../../..
fi

if [ ! -d arrow ]; then
    git clone --branch apache-arrow-19.0.1 --depth=1 https://github.com/apache/arrow.git
    cd arrow/cpp && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release -DARROW_BUILD_STATIC=ON -DARROW_DEPENDENCY_SOURCE=BUNDLED \
        -DARROW_ENABLE_THREADING=OFF -DARROW_FLIGHT=OFF -DARROW_JEMALLOC=OFF -DARROW_MIMALLOC=OFF -DARROW_BUILD_SHARED=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j $(nproc) && make install
    cd ../../..
fi

if [ ! -d boost_1_71_0 ]; then
    wget https://archives.boost.io/release/1.71.0/source/boost_1_71_0.tar.gz
    tar -xzf boost_1_71_0.tar.gz
    cd boost_1_71_0
    echo "using gcc : aarch64 : /usr/bin/aarch64-linux-gnu-g++-8.4.0 ;" > user-config.jam
    ./bootstrap.sh
    ./b2 --user-config=./user-config.jam toolset=gcc-aarch64 \
        --prefix=/usr/local/boost-aarch64 --build-dir=build-aarch64 \
        --with-system --with-filesystem --with-thread --with-date_time --with-chrono --with-regex \
        target-os=linux link=static runtime-link=static threading=multi \
        architecture=arm address-model=64 install
    cd ..
    rm boost_1_71_0.tar.gz
fi

if [ ! -d gtsam ]; then
    git clone --branch 4.2 --depth=1 https://github.com/borglab/gtsam.git
    cd gtsam && mkdir -p build && cd build && \
    cmake \
        -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_USE_SYSTEM_BOOST=ON \
        -DGTSAM_USE_SYSTEM_EIGEN=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j $(nproc) && make install
    cd ../..
fi

# if [ ! -d rerun ]; then
#     git clone --branch 0.22.1 --depth=1 https://github.com/rerun-io/rerun.git
#     curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
#     source $HOME/.cargo/env
#     rustup target add aarch64-unknown-linux-gnu
#     cd rerun && \
#     mkdir -p build && cd build && \
#     cmake -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake \
#         -DCMAKE_BUILD_TYPE=Release -DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF \
#         -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
#     make -j $(nproc) && make install
#     cd ../..
# fi

mkdir -p build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$DIRPATH/toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
make -j $(nproc)
cd ..

# after, run this command and make sure it's 3.4.29 or under:
# objdump -T build/IMX | grep GLIBCXX
