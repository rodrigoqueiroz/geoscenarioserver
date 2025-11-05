#!/bin/bash

set -e

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_ser
}
trap shutdown_nodes SIGINT SIGTERM EXIT

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd ${REPO_DIR}
pixi run ros_client_build_release
sleep 2  # Allow pixi to update build cache before launching dependent tasks
pixi run rqt_topic &
pixi run ros_mock_co_simulator --ros-args \
    -p target_delta_time:=0.025 \
    -p max_simulation_time:=10.0 \
    -p real_time_factor:=1.0 &

pixi run ros_native_gss --ros-args \
    --log-level INFO \
    -p dashboard_position:="[0.0, 0.0, 960.0, 1080.0]" \
    -p scenario_files:="['scenarios/long_test_scenarios/gs_ringroad_stress_loop.osm']"
