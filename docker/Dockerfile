# ROS 2 Humble build/test image for the openarm_ctc package (Ubuntu 22.04).
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-pinocchio \
      ros-humble-ament-cmake-gtest \
      ros-humble-launch-testing-ament-cmake \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
