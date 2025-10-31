#!/bin/bash

TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$TEST_DIR")

cd ${REPO_DIR}
# build the conda package
pixi build
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pixi build failed"
    exit 1
fi
mv geoscenarioserver-0.*.conda test/conda-test/geoscenarioserver.conda

# run the server in the test environment
cd test/conda-test
pixi install
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pixi install failed"
    exit 1
fi
pixi run gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: pixi run succeeded"
    # cleanup
    echo "$0: INFO: cleaning up"
    rm -rf .pixi/ outputs/
    rm -f geoscenarioserver.conda pixi.lock
    exit 0
else
    echo "$0: ERROR: pixi run failed"
    exit 1
fi


