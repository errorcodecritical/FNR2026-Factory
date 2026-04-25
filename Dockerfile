# ================================
# BASE IMAGE
# ================================

FROM ros:jazzy-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Fix mirrors
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/sources.list.d/ubuntu.sources

# Core build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    python3-yaml \
    python3-serial \
    ca-certificates \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ================================
# RUNTIME IMAGE (MAIN)
# ================================

FROM base AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    # Core localization
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-slam-toolbox \
    \
    # Navigation
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    \
    # Sensors
    ros-${ROS_DISTRO}-rplidar-ros \
    \
    # Teleop
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    \
    # TF tools
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-ros \
    \
    # Robot description
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    \
    # Visualization
    ros-${ROS_DISTRO}-rviz2 \
    \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Shared entrypoint
COPY ./docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]

WORKDIR /docker_ws
