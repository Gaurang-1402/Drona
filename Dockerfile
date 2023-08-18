FROM osrf/ros:humble-desktop as base
ENV rosdistro=humble
SHELL ["/bin/bash", "-c"]


# Create colcon workspace with dependencies
RUN mkdir /ros_ws
COPY ./src /ros_ws/src

# Set up rosdep
WORKDIR /ros_ws
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -y \
    && rosdep update \
    && rosdep install --rosdistro $rosdistro --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

# Source the workspace
RUN "source /ros_ws/install/setup.sh \
    && ros2 run rosgpt rosgpt \
    && ros2 run rosgpt rosgpt_client_node \
    && ros2 run rosgpt rosgptparser_drone"
