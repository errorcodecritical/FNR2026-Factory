FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install build dependencies with retry logic for transient mirror issues
RUN for i in 1 2 3; do \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libboost-all-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/* && break || ([ $i -lt 3 ] && sleep 10) || exit 1; done

# Copy CycloneDDS config
COPY cyclonedds.xml /cyclonedds.xml
ENV CYCLONEDDS_URI=file:///cyclonedds.xml

WORKDIR /docker_ws
