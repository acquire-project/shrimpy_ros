######################################################################################
# Dockerfile for building a ROS 2 image with waveorder_ros and rmw_zenoh

ARG ROS_DISTRO=rolling

# base runtime image containing rmw_zenoh
FROM ros:$ROS_DISTRO-ros-core AS ros-core-with-zenoh

# Install rmw_zenoh & other useful packages
RUN apt-get update && apt-get install -y \
        ros-$ROS_DISTRO-rmw-zenoh-cpp \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

######################################################################################
# build image based on ros-base (do not save)
FROM ros:$ROS_DISTRO-ros-base AS build

# copy local source into the image
COPY . /ros_ws/src
WORKDIR /ros_ws
RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select waveorder_ros --install-base=/ros_ws/install

######################################################################################
# final target image with zenoh and waveorder_ros
# Use the base image with zenoh
FROM ros-core-with-zenoh AS waveorder-ros

# Install waveorder-ros dependencies
RUN pip3 install --no-cache-dir --break-system-packages waveorder

# Copy the built waveorder_ros package from the build stage
COPY --from=build /ros_ws/install /opt/ros/custom

# COPY our custom entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh

