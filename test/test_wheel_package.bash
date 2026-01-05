#!/bin/bash

TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WHEEL_TEST_DIR=${TEST_DIR}/wheel-test
REPO_DIR=$(dirname "$TEST_DIR")

sudo apt-get install -y python3-dev python3-venv python3-tk

cd ${REPO_DIR}
# build the wheel package
pixi run build_wheel
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: wheel build failed"
    exit 1
fi
# run the server in the test environment
cd ${WHEEL_TEST_DIR}
python3 -m venv .venv
source .venv/bin/activate
WHEEL=$(realpath ${REPO_DIR}/dist/geoscenarioserver*.whl)
python3 -m pip install ${WHEEL}
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pip wheel install failed"
    exit 1
fi
gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: gsserver run succeeded"
    # cleanup
    echo "$0: INFO: cleaning up"
    rm -rf .venv/ outputs/
    exit 0
else
    echo "$0: ERROR: gsserver run failed"
    exit 1
fi


