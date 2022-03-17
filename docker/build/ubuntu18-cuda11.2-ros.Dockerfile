FROM nvidia/cuda:11.2.2-cudnn8-devel-ubuntu18.04 
# cuda-11.2, cudnn-8.1.1

### ----------------- Install ros ---------------------
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_PKG=desktop_full
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
COPY files/sources.list /etc/apt/
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    vim \
    cmake \
    build-essential \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    unzip \
    autoconf \
    automake \
    libtool \
    && rm -rf /var/lib/apt/lists/*
# add the ROS deb repo to the apt sources list
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-`echo "${ROS_PKG}" | tr '_' '-'` \
    && rm -rf /var/lib/apt/lists/*
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
RUN apt-get update -y && apt-get install -y \
    ros-melodic-jsk-recognition-msgs \
    ros-melodic-jsk-rviz-plugins \
    libgflags-dev \
    libgoogle-glog-dev \
    ros-melodic-serial \
    libatlas-base-dev \
    gdb \
    && rm -rf /var/lib/apt/lists/*

## eigen
COPY files/eigen/Core /usr/include/eigen3/Eigen/
COPY files/eigen/Half.h /usr/include/eigen3/Eigen/src/Core/arch/CUDA/
## 注释 common_functions.h 74行
COPY files/common_functions.h /usr/local/cuda/targets/x86_64-linux/include/crt/

## ------------------- CMake-3.20 -------------------
ADD https://cmake.org/files/v3.20/cmake-3.20.6-linux-x86_64.sh /cmake-3.20.6-linux-x86_64.sh
RUN mkdir /opt/cmake-3.20.6-linux-x86_64 && \
    sh /cmake-3.20.6-linux-x86_64.sh --prefix=/opt/cmake-3.20.6-linux-x86_64 --skip-license && \
    ln -s /opt/cmake-3.20.6-linux-x86_64/bin/cmake /usr/local/bin/cmake && \
    cmake --version && rm /cmake-3.20.6-linux-x86_64.sh

### ---------------- Install Protobuf -----------------
## install from source
RUN apt-get remove -y libprotoc-dev libprotobuf-dev && rm -rf /usr/include/google/protobuf
COPY files/protobuf-3.5.1.1.tar.gz /root
# RUN cd /root && wget -O protobuf-3.5.1.1.tar.gz https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.5.1.1.tar.gz
RUN tar xzf /root/protobuf-3.5.1.1.tar.gz -C /root && \
    cd /root/protobuf-3.5.1.1 && \
    ./autogen.sh && \
    ./configure --prefix=/usr && \
    make && \
    make check && \
    make install && \
    ldconfig && \
    rm -rf /protobuf-3.5.1.1.tar.gz /root/protobuf-3.5.1.1*

## --------------------- absl ---------------------
# RUN cd /root && git clone https://github.com/abseil/abseil-cpp.git
COPY files/abseil-cpp.tar.gz /root
RUN cd /root && tar -xzvf abseil-cpp.tar.gz && \
    cd abseil-cpp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_CXX_FLAGS=-fPIC && \
    cmake --build . --target install && \
    rm -rf /root/abseil-cpp*

### ----------------- Install TensorRT 8.2.3 -----------------
# RUN cd /root && wget https://developer.nvidia.com/compute/machine-learning/tensorrt/secure/8.2.3.0/local_repos/nv-tensorrt-repo-ubuntu1804-cuda11.4-trt8.2.3.0-ga-20220113_1-1_amd64.deb
COPY files/nv-tensorrt-repo-ubuntu1804-cuda11.4-trt8.2.3.0-ga-20220113_1-1_amd64.deb /root
RUN dpkg -i /root/nv-tensorrt-repo-ubuntu1804-cuda11.4-trt8.2.3.0-ga-20220113_1-1_amd64.deb
COPY files/local-repo /etc/apt/preferences.d/
RUN apt-get update -y && apt-get install -y tensorrt && \
    rm /root/nv-tensorrt* \
    && rm -rf /var/lib/apt/lists/*

## -------------------- libtorch 1.10.0 ------------------
COPY files/libtorch-cxx11-abi-shared-with-deps-1.10.0+cu113.zip /root
RUN cd /root && unzip libtorch-cxx11-abi-shared-with-deps-1.10.0+cu113.zip && \
    mv libtorch /usr/local/ && \
    rm -rf /root/libtorch*
