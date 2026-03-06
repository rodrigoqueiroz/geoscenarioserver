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
cp -f ${REPO_DIR}/geoscenarioserver-*.conda ${TEST_DIR}/conda-test/geoscenarioserver.conda

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
    git clean -fdx
else
    echo "$0: ERROR: pixi run failed"
    exit 1
fi

# micromamba
if [[ -z ${MAMBA_EXE} ]]; then
    echo "$0: micromamba not found, skipping the micromamba test"
    rm ${REPO_DIR}/geoscenarioserver-*.conda
    exit 0
fi

cd ${TEST_DIR}/conda-test
# ensure uv
${MAMBA_EXE} -n base install uv -y
mv -f ${REPO_DIR}/geoscenarioserver-*.conda ${TEST_DIR}/conda-test/channel/linux-64/
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
    git clean -fdx
    micromamba env remove -n gss -yq  
else
    echo "$0: ERROR: micromamba run failed"
    exit 1
fi
