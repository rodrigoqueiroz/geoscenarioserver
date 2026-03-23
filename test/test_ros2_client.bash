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
DISPLAY_OPTIONS="--dash-pos 0 0 960 1080 -wi"
if [[ -z ${DISPLAY} ]]; then
    DISPLAY_OPTIONS="--no-dash"
else 
    pixi run rqt_topic &
fi
pixi run ${ROS_CLIENT} &
pixi run ros_mock_co_simulator &

shutdown_nodes() {
    pkill --signal SIGTERM python
    pkill --signal SIGTERM mock_co_simulat
    pkill --signal SIGTERM geoscenario_cli
}
trap shutdown_nodes SIGINT SIGTERM EXIT

pixi run gsserver $DISPLAY_OPTIONS -s scenarios/test_scenarios/gs_all_vehicles_peds.osm
