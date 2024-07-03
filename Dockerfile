# Use an official Python runtime as a parent image
FROM ubuntu:22.04

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the working directory to /root
ENV DIRPATH /root
WORKDIR $DIRPATH

#Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y git cmake

RUN apt-get install -y tzdata

# Installing cross-compiling dependencies
RUN apt update -y && \
    apt upgrade -y

RUN apt install -y wget
RUN apt install -y gcc
RUN apt install -y make
RUN apt install -y cmake
RUN apt install -y build-essential
RUN apt install -y gcc-aarch64-linux-gnu
RUN apt install -y g++-aarch64-linux-gnu
RUN apt install -y binutils-aarch64-linux-gnu

# my additions
# --- for CLion Integration ---
RUN apt install -y gdb-multiarch
# --- for CLion Integration ---