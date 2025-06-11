# ---------- deps & third-party ----------
FROM ros:melodic-ros-base-bionic
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git cmake build-essential pkg-config \
    libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev \
    python2.7-dev python-numpy python-rosdep python-rosinstall python-wstool \
    libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libopenjp2-7-dev libdc1394-22-dev \
    libpcl-dev pcl-tools liboctomap-dev octovis && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-melodic-cv-bridge ros-melodic-image-geometry ros-melodic-image-transport \
    ros-melodic-octomap-server ros-melodic-grid-map-core ros-melodic-grid-map-msgs \
    ros-melodic-rviz ros-melodic-rviz-visual-tools ros-melodic-rosbag ros-melodic-grid-map-ros && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update --include-eol-distros

# Pangolin, Eigen, OpenCV
WORKDIR /opt
RUN git clone --depth 1 --branch v0.5 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && cmake .. && make -j$(nproc) && make install
RUN git clone --depth 1 --branch 3.1.0 https://gitlab.com/libeigen/eigen.git && \
    cd eigen && mkdir build && cd build && cmake .. && make install
RUN git clone --depth 1 --branch 3.4.15 https://github.com/opencv/opencv.git && \
    cd opencv && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DWITH_JASPER=OFF .. && \
    make -j$(nproc) && make install && ldconfig

# SG-SLAM third-party
WORKDIR /deps
COPY ./src/sg-slam/Thirdparty /deps/SG-SLAM/src/sg-slam/Thirdparty
WORKDIR /deps/SG-SLAM/src/sg-slam/Thirdparty
RUN ./ThirdpartyBuild.sh

# ncnn
WORKDIR /deps/SG-SLAM/src/sg-slam/Thirdparty/ncnn
RUN mkdir build && cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/host.gcc.toolchain.cmake -DNCNN_DISABLE_RTTI=OFF .. && \
    make -j$(nproc) && make install

COPY ./src /workspace/SG-SLAM/src
RUN cp -rf /deps/SG-SLAM/src/sg-slam/Thirdparty /workspace/SG-SLAM/src/sg-slam/
RUN rm -rf /deps/
ENV ncnn_DIR=/workspace/SG-SLAM/src/sg-slam/Thirdparty/ncnn/build/install/lib/cmake/ncnn

WORKDIR /workspace/SG-SLAM/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace"

WORKDIR /workspace/SG-SLAM
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

COPY src/sg-slam/configs /workspace/SG-SLAM/src/sg-slam/configs
ENV ROS_PACKAGE_PATH=/workspace/SG-SLAM/src

WORKDIR /workspace
CMD ["bash"]
