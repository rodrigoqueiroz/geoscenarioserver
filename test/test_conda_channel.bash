#!/bin/bash

TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$TEST_DIR")

# run the server in the test environment
cd ${TEST_DIR}/conda-channel-test
echo "--------"
echo "| pixi |"
echo "--------"
pixi install
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pixi install failed"
    exit 1
fi

# standalone
pixi run gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm --overlay-osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: pixi run succeeded"
    # ROS2
    pixi run ros2 run geoscenario_server geoscenario_server --ros-args \
                      -p heartbeat_period:=1.0 \
                      -p stall_muliplier:=1.0 \
                      -p scenario_files:=[scenarios/test_scenarios/gs_all_vehicles_peds.osm]
    # cleanup
    echo "$0: INFO: cleaning up"
    git clean -fdx
else
    echo "$0: ERROR: pixi run failed"
    exit 1
fi

# micromamba
if [[ -z ${MAMBA_EXE} ]]; then
    echo "$0: micromamba not found, skipping the micromamba test"
    exit 0
fi
echo "--------------"
echo "| micromamba |"
echo "--------------"
cd ${TEST_DIR}/conda-channel-test
# ensure uv
${MAMBA_EXE} -n base install uv -y
${MAMBA_EXE} env create -yq --use-uv -f conda-environment.yml 
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: micromamba create failed"
    exit 1
fi
# standalone
${MAMBA_EXE} run -n gss gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm --overlay-osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: micromamba run succeeded"
    # ROS2
    ${MAMBA_EXE} run -n gss ros2 run geoscenario_server geoscenario_server --ros-args  \
                                     -p heartbeat_period:=1.0 \
                                     -p stall_muliplier:=1.0 \
                                     -p scenario_files:=[scenarios/test_scen
    # cleanup
    echo "$0: INFO: cleaning up"
    git clean -fdx
    micromamba env remove -n gss -yq  
else
    echo "$0: ERROR: micromamba run failed"
    exit 1
fi
