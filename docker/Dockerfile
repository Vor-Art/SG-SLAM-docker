# ===== SG-SLAM development image =====
FROM ros:melodic-ros-base-bionic

ARG DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------------------
# 1) System packages + all ROS binary deps that SG-SLAM / rosdep need
# --------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git cmake build-essential vim pkg-config \
    libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev \
    python2.7-dev python-numpy python-rosdep python-rosinstall python-wstool \
    libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libopenjp2-7-dev libdc1394-22-dev \
    libpcl-dev pcl-tools \
    liboctomap-dev octovis && \
    rm -rf /var/lib/apt/lists/*

# rosdep: the base image is already initialised
RUN rosdep update --include-eol-distros

# Pangolin
WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && git checkout v0.5 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Eigen (3.1.0)
WORKDIR /opt
RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && git checkout 3.1.0 && mkdir build && cd build && \
    cmake .. && make install

# OpenCV 3.4.15 (depth-1 clone, no Jasper)
WORKDIR /opt
RUN git clone --depth 1 --branch 3.4.15 https://github.com/opencv/opencv.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -DWITH_JASPER=OFF .. && \
    make -j$(nproc) && make install && ldconfig

# Copy local checkout instead of cloning
WORKDIR /workspace
COPY ./src /workspace/SG-SLAM/src
RUN cd SG-SLAM/src/sg-slam && ./ThirdpartyBuild.sh

# ncnn — RTTI must stay ON
WORKDIR /workspace/SG-SLAM/src/sg-slam/Thirdparty/ncnn
RUN mkdir build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/host.gcc.toolchain.cmake -DNCNN_DISABLE_RTTI=OFF .. && \
    make -j$(nproc) && make install
ENV ncnn_DIR=/workspace/SG-SLAM/src/sg-slam/Thirdparty/ncnn/build/install/lib/cmake/ncnn


RUN apt-get update && apt-get install -y \
    ros-melodic-cv-bridge \
    ros-melodic-image-geometry \
    ros-melodic-image-transport \
    ros-melodic-octomap-server \
    ros-melodic-grid-map-core \
    ros-melodic-grid-map-msgs \
    ros-melodic-rviz \
    ros-melodic-rviz-visual-tools \
    ros-melodic-rosbag \
    ros-melodic-grid-map-ros && \
    rm -rf /var/lib/apt/lists/*

# Catkin workspace init + one-shot build
WORKDIR /workspace/SG-SLAM/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace"

WORKDIR /workspace/SG-SLAM
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"


# Runtime environment
ENV ROS_PACKAGE_PATH=/workspace/SG-SLAM/src
ENV QT_X11_NO_MITSHM=1
ENV QT_QPA_PLATFORM=xcb

CMD ["bash"]

