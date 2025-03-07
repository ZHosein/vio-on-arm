set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a") # Prefer static libraries

set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc-8)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-8)
set(CMAKE_AR /usr/bin/aarch64-linux-gnu-ar)
set(CMAKE_LINKER /usr/bin/aarch64-linux-gnu-ld)

set(RUST_TARGET "aarch64-unknown-linux-gnu")
set(CARGO_TARGET_AARCH64_LINUX_GNU_LINKER /usr/bin/aarch64-linux-gnu-gcc-8)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
