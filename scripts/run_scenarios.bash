#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

ARG_VEHICLES="true"
ARG_LONG="false"
ARG_PEDESTRIANS="true"
ARG_NO_DASH=

print_help() {
    echo "Running all vehicle and pedestrian scenarios except long scenarios by default."
    echo "  --op        run only pedestrian scenarios"
    echo "  --ov        run only vehicle scenarios"
    echo "  --long      include long vehicle scenarios (not included by default)"
    echo "  --no-dash   run without the dashboard."
    echo ""
}

if [[ "$#" -eq 0 ]]; then
    print_help
else
    for arg in "$@"; do
        case $arg in
            "--op")
                ARG_VEHICLES="false"
                ;;
            "--ov")
                ARG_PEDESTRIANS="false"
                ;;
            "--long")
                ARG_LONG="true"
                ;;
            "--no-dash")
                ARG_NO_DASH="--no-dash"
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
long_test_scenarios=
if [[ "$ARG_LONG" == "true" ]]; then
    long_test_scenarios=$(find "${REPO_DIR}/scenarios/long_test_scenarios" -name "*.osm" | sort)
fi
all_scenarios="$test_scenarios $pedestrian_scenarios $long_test_scenarios"

kill_python3()
{
    killall python3
}

cd ${REPO_DIR}
for scenario in $all_scenarios; do
    echo "CTRL + C to exit the script."
    read -p "ENTER to run ${scenario#$REPO_DIR/}:"

    trap kill_python3 SIGINT
    echo "CTRL + C to quit the scenario."
    ${MAMBA_EXE} -n gss run python3 GSServer.py ${ARG_NO_DASH} --scenario ${scenario}
    echo "=== violations.json for \"$(basename ${scenario})\" ==="
    cat ${REPO_DIR}/outputs/violations.json
    echo ""
    echo "==="
    trap - SIGINT
done
