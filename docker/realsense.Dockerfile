# ── RealSense streamer (Ubuntu packages) ─────────────────────────────
FROM ros:melodic-ros-base-bionic

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl udev ca-certificates gnupg2 \
      ros-melodic-realsense2-camera ros-melodic-realsense2-description \
      ros-melodic-rviz ros-melodic-rviz-visual-tools \
 && rm -rf /var/lib/apt/lists/*

CMD ["roslaunch", "realsense2_camera", "rs_camera.launch", "align_depth:=true"]
