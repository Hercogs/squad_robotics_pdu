# variables for build-time
ARG ROS_DISTRIBUTION=humble

ARG package_name=squad_robotics_pdu

# use ros2 docker image as a base
FROM ros:${ROS_DISTRIBUTION}-ros-base

# Install clang and set as default compiler.
RUN apt-get update && apt-get install -y --no-install-recommends \
  clang \
  && rm -rf /var/lib/apt/lists/*

# Remove packages

# Set environment and working directory
ENV ROS_WS /ros2_ws
ENV ROS_WS_SRC /ros2_ws/src
WORKDIR $ROS_WS_SRC

# RUN mkdir -p $ROS_WS_SRC/$package_name
COPY ./ $ROS_WS_SRC/$package_name/

# Set up packages
WORKDIR $ROS_WS_SRC/$package_name/PDU_ros2_node
RUN apt-get install ./ros-humble-robot-*jammy_amd64.deb

# Install rosdep dependencies
WORKDIR $ROS_WS
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash"
RUN apt-get update && rosdep update --include-eol-distros && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build workspace
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash" \
#     && colcon build
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build

# source workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$ROS_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "squad_robotics_pdu", "launch_pdu.launch.py"]


# docker build -t squad_robotics_pdu -f Dockerfile .

# docker run --rm -it --privileged -e ROBOT_NAME=$ROBOT_NAME -v /dev:/dev  squad_robotics_pdu

