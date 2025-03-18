#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd ${REPO_DIR}
pixi run -e humble build_ros_client
pixi run -e humble start_rqt &
pixi run -e humble start_ros_client &
pixi run -e humble start_ros_mock_co_simulator &

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_cli
}
trap shutdown_nodes SIGINT SIGTERM EXIT

pixi run start_gss scenarios/test_scenarios/gs_ringroad_stress_loop.osm
