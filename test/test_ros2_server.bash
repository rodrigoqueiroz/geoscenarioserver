#!/bin/bash
set -e

if [ "$1" == "--help" ]; then
    echo "Usage: $0 [--fastest|--10xrealtime|--2xrealtime] [--dev] [scenario_file]"
    echo "  Runs the ROS2 GeoScenarioServer with the mock co-simulator in real-time (default) or faster modes:"
    echo "  --fastest : Run the test with the as fast as possible time control, runs simulation till scenario completion"
    echo "  --10xrealtime : Run the test with 10x faster than real-time for max 30 simulated seconds"
    echo "  --2xrealtime : Run the test with 2x faster than real-time for max 30 simulated seconds"
    echo "  --dev : Run the test in humble-dev environment (with colcon) (runs in humble environment by default)"
    echo "  [scenario_file] : Optional scenario file to use (gs_all_vehicles_peds.osm by default)"
    exit 1
fi  

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_ser
}
trap shutdown_nodes SIGINT SIGTERM EXIT

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

MAX_SIM_TIME=30.0
DEV=""
TIME_FACTOR=1.0
SCENARIO_FILE=
for arg in "$@"; do
    case "$arg" in
        --dev)
            DEV="_dev"
            echo "Running in humble-dev environment with colcon build."
            pixi run ros_build
            ;;
        --fastest)
            TIME_FACTOR=0.0
            MAX_SIM_TIME=-1.0
            ;;
        --2xrealtime)
            TIME_FACTOR=0.5
            ;;
        --10xrealtime)
            TIME_FACTOR=0.1
            ;;
        *)
            SCENARIO_FILE=${arg}
            echo "Running in real-time mode."
            ;;
    esac
done
if [ -z "$SCENARIO_FILE" ]; then
    SCENARIO_FILE="scenarios/test_scenarios/gs_all_vehicles_peds.osm"
fi

cd ${REPO_DIR}
set -x

pixi run rqt_topic${DEV} &

pixi run ros_mock_co_simulator${DEV} --ros-args \
        -p target_delta_time:=0.025 \
        -p max_simulation_time:=${MAX_SIM_TIME} \
        -p real_time_factor:=${TIME_FACTOR} &

pixi run ros_server${DEV} --ros-args \
    --log-level INFO \
    -p dashboard_position:="[0, 0, 1920, 1080]" \
    -p scenario_files:="['$SCENARIO_FILE']"


echo "Sleeping 5s to allow the nodes to shutdown..."
sleep 5
echo "Shutdown complete."