#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

ARG_VEHICLES="true"
ARG_PEDESTRIANS="true"

print_help() {
    echo "Running all vehicle and pedestrian scenarios by default."
    echo "  Use -op to run only pedestrian scenarios."
    echo "  Use -ov to run only vehicle scenarios."
    echo ""
}

if [[ "$#" -eq 0 ]]; then
    print_help
else
    for arg in "$@"; do
        case $arg in
            "-op")  # Only pedestrians
                ARG_VEHICLES="false"
                ;;
            "-ov")
                ARG_PEDESTRIANS="false"
                ;;
            *)
                echo "Invalid argument $arg"
                echo ""
                echo "Usage: "
                echo ""
                print_help
                exit 1
        esac
    done
fi

test_scenarios=""
if [[ "$ARG_VEHICLES" == "true" ]]; then
    test_scenarios=$(find "${REPO_DIR}/scenarios/test_scenarios" -name "*.osm" | sort)
fi
pedestrian_scenarios=""
if [[ "$ARG_PEDESTRIANS" == "true" ]]; then
    pedestrian_scenarios=$(find "${REPO_DIR}/scenarios/pedestrian_scenarios" -name "*.osm" | sort)
fi
scenarios="$test_scenarios $pedestrian_scenarios"

source "${REPO_DIR}/catkin_ws/install/opt/ros/lanelet2/setup.bash" --extend

kill_python38()
{
    killall python3.8
}

for scenario in $scenarios; do
    echo "CTRL + C to exit the script."
    read -p "ENTER to run ${scenario#$REPO_DIR/}:"

    trap kill_python38 SIGINT
    echo "CTRL + C to quit the scenario."
    python3.8 GSServer.py -s $scenario
    trap - SIGINT
done
