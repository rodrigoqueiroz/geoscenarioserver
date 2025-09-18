#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")
mkdir -p ${REPO_DIR}/colcon_ws/src
ln -sfn ${REPO_DIR}/clients/ros2_client ${REPO_DIR}/colcon_ws/src/ros2_client
cd ${REPO_DIR}/colcon_ws
colcon build $@
