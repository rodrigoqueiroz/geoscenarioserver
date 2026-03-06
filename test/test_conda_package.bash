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

# for pixi
cp -f geoscenarioserver-*.conda ${TEST_DIR}/conda-test/geoscenarioserver.conda
# for micromamba
mv -f geoscenarioserver-*.conda ${TEST_DIR}/conda-test/channel/linux-64/

# run the server in the test environment
cd ${TEST_DIR}/conda-test
# pixi
pixi install
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: pixi install failed"
    exit 1
fi

pixi run gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm --overlay-osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: pixi run succeeded"
    # cleanup
    echo "$0: INFO: cleaning up"
    rm -rf .pixi/ outputs/
    rm -f geoscenarioserver.conda pixi.lock
else
    echo "$0: ERROR: pixi run failed"
    exit 1
fi

# micromamba
if [[ -z ${MAMBA_EXE} ]]; then
    echo "$0: micromamba not found, skipping micromamba test"
    exit 0
fi
cd ${TEST_DIR}/conda-test
# ensure uv
${MAMBA_EXE} -n base install uv -y
pixi exec rattler-index fs ${TEST_DIR}/conda-test/channel
${MAMBA_EXE} env create -yq --use-uv -f conda-environment.yml 
if [[ $? -ne 0 ]]; then
    echo "$0: ERROR: micromamba create failed"
    exit 1
fi
${MAMBA_EXE} run -n gss gsserver -s scenarios/test_scenarios/gs_all_vehicles_peds.osm --overlay-osm
if [[ $? == 0 ]]; then
    echo "$0: INFO: micromamba run succeeded"
    # cleanup
    echo "$0: INFO: cleaning up"
    rm -rf outputs/
    micromamba env remove -n gss -yq
    rm -rf ${TEST_DIR}/conda-test/channel/linux-64/.cache/
    rm -rf ${TEST_DIR}/conda-test/channel/linux-64/shards/
    rm -f ${TEST_DIR}/conda-test/channel/linux-64/*
    rm -rf ${TEST_DIR}/conda-test/channel/noarch/
else
    echo "$0: ERROR: micromamba run failed"
    exit 1
fi
