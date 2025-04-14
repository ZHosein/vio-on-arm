# Use an official Python runtime as a parent image
FROM ubuntu:20.04

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the working directory to /root
#TODO: change to "ENV DIRPATH=/root
ENV DIRPATH=/root
WORKDIR $DIRPATH
COPY . .

#Install build dependencies
RUN bash build.sh
