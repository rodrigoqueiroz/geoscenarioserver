#!/bin/bash

TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WHEEL_TEST_DIR=${TEST_DIR}/wheel-test
REPO_DIR=$(dirname "$TEST_DIR")

sudo apt-get install -y python3-venv python3-pip

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
python3 -m pip install ${REPO_DIR}/dist/geoscenarioserver-0.1.0-py3-none-any.whl
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
    rm -f geoscenarioserver.whl
    exit 0
else
    echo "$0: ERROR: gsserver run failed"
    exit 1
fi


