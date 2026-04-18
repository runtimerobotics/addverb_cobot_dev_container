FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
USER root

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install base tools, locale, and apt sources for ROS 2 Humble and Gazebo
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    sudo \
    tzdata \
    software-properties-common \
    wget && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    mkdir -p /etc/apt/keyrings /var/lib/apt/lists/partial && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /etc/apt/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS 2 Humble + Nav2 + MoveIt2 (from apt) + Gazebo Classic 11 + ros2_control stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
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
    ros-humble-pointcloud-to-laserscan \
    ros-humble-usb-cam \
    ros-humble-rqt-image-view \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
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
    # Optional packages that may not be present on all architectures
    for pkg in \
      ros-humble-ur-simulation-gazebo \
      ros-humble-ros-ign-bridge \
      ros-humble-ros-ign-gazebo; do \
      if apt-cache show "$pkg" >/dev/null 2>&1; then \
        apt-get install -y --no-install-recommends "$pkg"; \
      else \
        echo "Skipping unavailable package: $pkg"; \
      fi; \
    done && \
    rosdep init || true && \
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
    '[ -f /home/robot/cobot_ros2_ws/install/setup.bash ] && source /home/robot/cobot_ros2_ws/install/setup.bash' \
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

# Clone cobot_ros2, install backend to /opt/addverb, then build workspace
# Per README: https://github.com/HumanoidAddverb/cobot_ros2
RUN mkdir -p ~/cobot_ros2_ws/src && \
    cd ~/cobot_ros2_ws/src && \
    git clone --depth=1 https://github.com/HumanoidAddverb/cobot_ros2.git && \
    sudo cp -r cobot_ros2/cobot_backend /opt/addverb/ && \
    rm -rf cobot_ros2/cobot_backend

RUN source /opt/ros/humble/setup.bash && \
    cd ~/cobot_ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
      --skip-keys "warehouse_ros_mongo haptic_pkg" && \
    colcon build --symlink-install

RUN printf '%s\n' \
    '# ROS 2 Humble' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f ~/cobot_ros2_ws/install/setup.bash ] && source ~/cobot_ros2_ws/install/setup.bash' \
    'export ROS_DOMAIN_ID=0' \
    >> ~/.bashrc

CMD ["/bin/bash"]
