#!/bin/bash

TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WHEEL_TEST_DIR=${TEST_DIR}/wheel-test
REPO_DIR=$(dirname "$TEST_DIR")

sudo apt-get install -yq python3-dev python3-venv python3-tk

if [[ $1 == "--release" ]]; then
    VERSION="0.1.1"
    WHEEL=https://github.com/rodrigoqueiroz/geoscenarioserver/releases/download/v${VERSION}/geoscenarioserver-${VERSION}-py3-none-any.whl
else
    cd ${REPO_DIR}
    # build the wheel package
    pixi run build_wheel
    if [[ $? -ne 0 ]]; then
        echo "$0: ERROR: wheel build failed"
        exit 1
    fi
    WHEEL=$(realpath ${REPO_DIR}/dist/geoscenarioserver*.whl)
fi

# run the server in the test environment
cd ${WHEEL_TEST_DIR}
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install ${WHEEL}
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pip wheel install failed"
    exit 1
fi
DISPLAY_OPTIONS="--overlay-osm"
if [[ -z ${DISPLAY} ]]; then
    DISPLAY_OPTIONS="--no-dash"
fi
gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm $DISPLAY_OPTIONS
if [[ $? == 0 ]]; then
    echo "$0: INFO: gsserver run succeeded"
    # cleanup
    echo "$0: INFO: cleaning up"
    git clean -fdx
    exit 0
else
    echo "$0: ERROR: gsserver run failed"
    exit 1
fi


