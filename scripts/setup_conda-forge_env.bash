#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

if [[ `which mamba` ]]; then
    if ! mamba env list | grep -q gss; then
        mamba create -n gss python=3.10
    fi
    mamba activate gss
    mamba install antlr4-python3-runtime==4.9.3 glog matplotlib numpy scipy tk 
    pip install antlr-denter lanelet2 py_trees==0.7.6 sysv-ipc

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