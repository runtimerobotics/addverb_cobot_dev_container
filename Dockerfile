FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]
USER root

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install additional base tools and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    tzdata \
    software-properties-common \
    wget \
    ros-humble-moveit \
    ros-humble-ur \
    ros-humble-urdf-tutorial \
    ros-humble-rqt-robot-steering \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-msgs \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-effort-controllers \
    ros-humble-xacro \
    ros-humble-launch-ros \
    ros-humble-ackermann-msgs \
    liborocos-kdl-dev \
    ros-humble-usb-cam \
    ros-humble-rqt-image-view \
    python3-colcon-common-extensions \
    python-is-python3 \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    vim \
    nano \
    curl \
    jq && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# System-wide ROS 2 sourcing for all login shells (including VSCode devcontainer terminals)
RUN printf '%s\n' \
    '#!/bin/bash' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /home/robot/cobot_ros2ws/install/setup.bash ] && source /home/robot/cobot_ros2ws/install/setup.bash' \
    > /etc/profile.d/ros2.sh && \
    chmod +x /etc/profile.d/ros2.sh

# Create non-root robot user
RUN useradd -m -s /bin/bash robot && \
    echo "robot ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/robot && \
    chmod 0440 /etc/sudoers.d/robot

# Create /opt/addverb for cobot backend
RUN mkdir -p /opt/addverb

USER robot
WORKDIR /home/robot

# Clone from GitHub to extract backend, then copy local cobot_ros2 as the build source
RUN mkdir -p ~/cobot_ros2ws/src && \
    cd ~/cobot_ros2ws/src && \
    git clone --depth=1 --branch heal https://github.com/HumanoidAddverb/cobot_ros2.git && \
    sudo cp -r cobot_ros2/cobot_backend /opt/addverb/ && \
    rm -rf cobot_ros2

COPY --chown=robot:robot cobot_ros2 /home/robot/cobot_ros2ws/src/cobot_ros2

RUN source /opt/ros/humble/setup.bash && \
    cd ~/cobot_ros2ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
      --skip-keys "warehouse_ros_mongo haptic_pkg" && \
    colcon build --symlink-install

RUN printf '%s\n' \
    '# ROS 2 Humble' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f ~/cobot_ros2ws/install/setup.bash ] && source ~/cobot_ros2ws/install/setup.bash' \
    'export ROS_DOMAIN_ID=0' \
    >> ~/.bashrc

CMD ["/bin/bash"]
