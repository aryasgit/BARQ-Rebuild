FROM dustynv/ros:humble-desktop-l4t-r36.4.0

ENV DEBIAN_FRONTEND=noninteractive

# Fix expired ROS apt key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 core + control + description + comms packages
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-state-broadcaster \
    ros-humble-forward-command-controller \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-serial-driver \
    ros-humble-topic-tools \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-transforms3d \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --index-url https://pypi.org/simple/ \
    mujoco \
    numpy \
    scipy \
    matplotlib \
    transforms3d \
    pyyaml \
    pyserial

# Init rosdep
RUN rosdep init || true && rosdep update

# Source ROS on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Gazebo Fortress (the Humble-paired release) + ros2_control plugin (Stage 2E).
# Appended as a late layer so earlier layers stay cached.
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    # NOT the ros-gz metapackage: its demos/image extras pull Ubuntu libopencv-dev, which
    # collides with the dustynv image's CUDA opencv-dev. Slim set only:
    apt-get install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge && \
    (apt-get install -y ros-humble-ign-ros2-control || apt-get install -y ros-humble-gz-ros2-control) && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/barq_ws