#!/bin/bash
set -ex

# Source ROS2 environment from host dependencies
source $PREFIX/setup.bash

# Build all ROS2 packages
# Use separate build directory to avoid conflicts with existing build artifacts
colcon build \
  --base-paths src \
  --packages-select geoscenario_msgs geoscenario_server geoscenario_client geoscenario_bringup \
  --build-base conda_build \
  --install-base $PREFIX \
  --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
