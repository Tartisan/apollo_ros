FROM nvidia/cuda:10.0-ros-melodic-ubuntu18.04

### ---------------- Install cudnn --------------------
COPY files/cudnn-10.0-linux-x64-v7.5.0.56.tgz /root 
RUN tar -xf /root/cudnn-10.0-linux-x64-v7.5.0.56.tgz -C /root &&\
    cp /root/cuda/include/*.h /usr/local/cuda/include/ &&\
    cp /root/cuda/lib64/libcudnn* /usr/local/cuda/lib64/ &&\
    rm -rf /root/cuda && \
    rm /root/cudnn-10.0-linux-x64-v7.5.0.56.tgz

### ------------------  opencv3.3 ---------------------
COPY files/opencv-3.3.1.tar.gz /root
RUN tar -xzvf /root/opencv-3.3.1.tar.gz -C /root && \ 
    mkdir -p /root/opencv-3.3.1/build && \
    cd /root/opencv-3.3.1/build && \
    cmake -D CMAKE_BUILD_TYPE=Release  -D CMAKE_INSTALL_PREFIX=/usr/local/ .. && \
    make && \
    make install && \
    touch /etc/ld.so.conf.d/opencv.conf && \
    echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf && \
    ldconfig && \
    echo "export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" >> /etc/bash.bashrc && \
    /bin/bash -c "source /etc/bash.bashrc" && \
RUN apt-get update -y && apt-get install -y locate && 
    updatedb && \ 
    cd /root && rm -rf /root/opencv*

### ----------------- Install TensorRT ----------------
COPY files/nv-tensorrt-repo-ubuntu1804-cuda10.0-trt5.1.5.0-ga-20190427_1-1_amd64.deb /root
RUN dpkg -i /root/nv-tensorrt-repo-ubuntu1804-cuda10.0-trt5.1.5.0-ga-20190427_1-1_amd64.deb && \
    apt-get update -y && apt-get install -y \
    libnvinfer5=5.1.5-1+cuda10.0 \
    libnvinfer-dev=5.1.5-1+cuda10.0 \
    libnvinfer-samples=5.1.5-1+cuda10.0 
RUN apt-get install -y tensorrt && \
    rm /root/nv-tensorrt* && \
    rm -rf /var/lib/apt/lists/*

### ---------------- Install Protobuf -----------------
## install from source
RUN apt-get remove libprotoc-dev libprotobuf-dev && \
    rm -rf /usr/include/google/protobuf
COPY files/protobuf-3.5.1.1.tar.gz /root 
RUN tar -xf /root/protobuf-3.5.1.1.tar.gz -C /root && \
    cd /root/protobuf-3.5.1.1 && \
    ./autogen.sh && \
    ./configure --prefix=/usr && \
    make && \
    make check && \
    make install && \
    ldconfig && \
    rm -rf /root/protobuf-3.5.1.1*

### ----------------fix bug while compiling avos3.0------------------
### 1. install gflags, serial, glog
### 2. solve libcudnn.so.7 is not a symbolic link
### 3. add c++11 for toplevel.cmake 
### 4. error: template with C linkage
RUN apt-get update -y && apt-get install -y \
    libgflags-dev \
    ros-melodic-serial \
    libgoogle-glog-dev \
    libatlas-base-dev \
    gdb && \
    ln -sf /usr/local/cuda-10.0/targets/x86_64-linux/lib/libcudnn.so.7.0.5 \
    /usr/local/cuda-10.0/targets/x86_64-linux/lib/libcudnn.so.7 && \
    rm -rf /var/lib/apt/lists/* && \
    rm /opt/ros/melodic/share/catkin/cmake/toplevel.cmake && \
    rm /usr/local/cuda-10.0/include/nppdefs.h && \
    rm /usr/local/cuda-10.0/include/nppi.h
COPY files/toplevel.cmake /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
COPY files/nppdefs.h /usr/local/cuda-10.0/include/ 
COPY files/nppi.h /usr/local/cuda-10.0/include/ 

## 修改 bashrc 里的 ROS_MASTER_URI
## /usr/include/eigen3/Eigen/Core 42行改为 #include <cuda_runtime.h>
## 注释 /usr/local/cuda-10.0/targets/x86_64-linux/include/crt/common_functions.h 74行

# libtorch-cxx11-abi-shared-with-deps-1.7.0+cu101

# cmake-3.20.6-linux-x86_64