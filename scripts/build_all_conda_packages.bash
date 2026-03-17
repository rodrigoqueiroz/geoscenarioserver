#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd ${REPO_DIR}
pixi build
# creates geoscenarioserver-<version>-<build>.conda

pixi build --path ros2/src/geoscenario_msgs/package.xml
# Creates ros-humble-geoscenario-msgs-<version>-<build>.conda

pixi build --path ros2/src/geoscenario_server/package.xml
# Creates ros-humble-geoscenario-server-<version>-<build>.conda

pixi build --path ros2/src/geoscenario_client/package.xml
# Creates ros-humble-geoscenario-client-<version>-<build>.conda
