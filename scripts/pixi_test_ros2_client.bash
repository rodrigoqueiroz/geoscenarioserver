#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

ROS_CLIENT="ros_client"
if [[ "$1" == "--wgs84" ]]; then
    ROS_CLIENT="${ROS_CLIENT}_wgs84"
elif [[ "$1" == "--roundtriptest" ]]; then
    ROS_CLIENT="${ROS_CLIENT}_wgs84_roundtriptest"
fi

cd ${REPO_DIR}
pixi run -e humble ros_client_build
pixi run -e humble rqt &
pixi run -e humble ${ROS_CLIENT} &
pixi run -e humble ros_mock_co_simulator &

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_cli
}
trap shutdown_nodes SIGINT SIGTERM EXIT

pixi run gss --dash-pos 0 0 960 1080 -wi -s scenarios/long_test_scenarios/gs_ringroad_stress_loop.osm
