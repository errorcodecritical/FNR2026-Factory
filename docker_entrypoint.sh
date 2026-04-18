#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Extract package name from first argument
PACKAGE_NAME="${1:?Package name required as first argument}"
shift  # Remove the package name, shift remaining args to $@

# Build the specific package in docker_ws if source code exists
if [ -d "/docker_ws/src/${PACKAGE_NAME}" ]; then
    echo "[docker_entrypoint] Building package: ${PACKAGE_NAME}"
    cd /docker_ws
    colcon build --packages-select ${PACKAGE_NAME} --symlink-install
else
    echo "[docker_entrypoint] Package source not found at /docker_ws/src/${PACKAGE_NAME}, skipping build"
fi

# Source the docker container workspace if it exists
if [ -d "/docker_ws/install" ]; then
    source /docker_ws/install/setup.bash
fi

# If the first remaining arg is bash, we need to ensure the environment is sourced in the subshell
if [ "$1" = "bash" ] && [ "$2" = "-c" ]; then
    # Inject sourcing into the bash command
    shift 2  # Remove 'bash' and '-c'
    exec bash -c "source /opt/ros/jazzy/setup.bash && if [ -d /docker_ws/install ]; then source /docker_ws/install/setup.bash; fi && $*"
else
    # Execute the provided command directly
    exec "$@"
fi

