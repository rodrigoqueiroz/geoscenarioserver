#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

test_scenarios=$(find "${REPO_DIR}/scenarios/test_scenarios" -name "*.osm" | sort)
pedestrian_scenarios=$(find "${REPO_DIR}/scenarios/pedestrian_scenarios" -name "*.osm" | sort)
scenarios="$test_scenarios $pedestrian_scenarios"

source "${REPO_DIR}/catkin_ws/install/opt/ros/lanelet2/setup.bash" --extend

kill_python38()
{
    killall python3.8
}

for scenario in $scenarios; do
    echo "CTRL + C to the script."
    read -p "ENTER to run ${scenario#$REPO_DIR/}:"

    trap kill_python38 SIGINT
    echo "CTRL + C to quit the scenario."
    python3.8 GSServer.py -s $scenario
    trap - SIGINT
done

