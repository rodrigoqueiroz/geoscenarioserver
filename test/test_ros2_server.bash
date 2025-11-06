#!/bin/bash
set -e

if [ -z $1 ]; then
    echo "Usage: $0 [--fastest|--realtime|--2xrealtime] [scenario_file]"
    echo "  --fastest : Run the test with the as fast as possible time control, runs simulation till scenario completion"
    echo "  --realtime : Run the test in real-time time for max 30 simualted seconds"
    echo "  --2xrealtime : Run the test with 2x faster than real-time for max 30 simualted seconds"
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

MAX_SIM_TIME=30.0

if [ "$1" == "--fastest" ]; then
    TIME_FACTOR=0.0
    MAX_SIM_TIME=-1.0
elif [ "$1" == "--2xrealtime" ]; then
    TIME_FACTOR=0.5
else
    TIME_FACTOR=1.0
fi

cd ${REPO_DIR}
pixi run ros_build_release
sleep 2  # Allow pixi to update build cache before launching dependent tasks
pixi run rqt_topic &

pixi run ros_mock_co_simulator --ros-args \
        -p target_delta_time:=0.025 \
        -p max_simulation_time:=${MAX_SIM_TIME} \
        -p real_time_factor:=${TIME_FACTOR} &

pixi run ros_server --ros-args \
    --log-level INFO \
    -p dashboard_position:="[0.0, 0.0, 1920.0, 1080.0]" \
    -p scenario_files:="['$SCENARIO_FILE']"
