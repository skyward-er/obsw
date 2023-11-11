FROM ubuntu:20.04

# Configure timezone
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install required packages
RUN apt-get update
RUN apt-get install -y sudo
RUN apt-get install -y build-essential
RUN apt-get install -y cmake 
RUN apt-get install -y ninja-build 
RUN apt-get install -y ccache 
RUN apt-get install -y xz-utils

# Setup Miosix
ADD https://miosix.org/toolchain/MiosixToolchainInstaller.run MiosixToolchainInstaller.run
RUN sh MiosixToolchainInstaller.run
