#!/bin/bash
set -e

if [ -z $1 ]; then
    echo "Usage: $0 <fastest|--realtime|--2xrealtime> [scenario_file]"
    echo "  --fastest : Run the test with the as fast as possible time control"
    echo "  --realtime : Run the test in real-time time"
    echo "  --2xrealtime : Run the test with 2x faster than real-time"
    echo "  [scenario_file] : Optional scenario file to use (gs_all_vehicles_peds.osm by default)"
    exit 1
fi  

SCENARIO_FILE=${2:-scenarios/test_scenarios/gs_all_vehicles_peds.osm}

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

if [ "$1" == "--fastest" ]; then
    pixi run ros_mock_co_simulator --ros-args \
        -p max_simulation_time:=30.0 &
else
    TIME_FACTOR=1.0
    if [ "$1" == "--2xrealtime" ]; then
        TIME_FACTOR=0.5
    fi
    pixi run ros_mock_co_simulator --ros-args \
        -p target_delta_time:=0.025 \
        -p max_simulation_time:=30.0 \
        -p real_time_factor:=${TIME_FACTOR} &
fi
pixi run ros_native_gss --ros-args \
    --log-level INFO \
    -p dashboard_position:="[0.0, 0.0, 1920.0, 1080.0]" \
    -p scenario_files:="['$SCENARIO_FILE']"
