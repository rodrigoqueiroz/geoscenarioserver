#!/bin/bash

set -e

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_ser
}

# setup the trap early, so it works even if we exit early. set -e exits if any errors occur
trap shutdown_nodes SIGINT SIGTERM EXIT

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd ${REPO_DIR}
pixi run ros_client_build_release
pixi run rqt_topic &
pixi run ros_mock_co_simulator &

pixi run ros_gss --ros-args \
    --log-level INFO \
    -p dashboard_position:="[0.0, 0.0, 960.0, 1080.0]" \
    -p scenario_files:="['scenarios/long_test_scenarios/gs_ringroad_stress_loop.osm']"
