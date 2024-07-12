#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

if [[ `which mamba` ]]; then
    if ! mamba env list | grep -q gss; then
        mamba env create --file ${SCRIPT_DIR}/conda-environment.yml
    fi
    mamba activate gss
    cd $REPO_DIR
    (set -x; ./GSServer.py -s scenarios/coretest_scenarios/straightdrive.gs.osm)

    echo "Running GeoScenario server within miniforge gss environement"
    echo "Execute:"
    echo "mamba activate gss"
    echo "cd geoscenarioserver"
    echo "./GSServer.py -s <scenario path>"
    echo ""
else 
    echo "Miniforge/mamba not installed"
    echo "Follow the instructions at https://github.com/conda-forge/miniforge"
    echo ""
    exit 1
fi