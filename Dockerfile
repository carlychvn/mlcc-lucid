FROM ros:noetic-robot-focal

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    libceres-dev \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Set up the workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make

# Default command
CMD ["/bin/bash"]
