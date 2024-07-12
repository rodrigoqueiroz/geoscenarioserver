#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

# Use MAMBA_EXE variable so that it works with either micromamba or mamba
if [[ -z $MAMBA_EXE ]]; then
    echo "Mamba or micromamba not installed or not activated."
    echo "Follow the instructions for installing either"
    echo "   micromamba (recommended): https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html"
    echo "   or mamba:                 https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html"
    exit 1
fi

if ! $MAMBA_EXE env list | grep -q gss; then
    $MAMBA_EXE env create --file ${SCRIPT_DIR}/conda-environment.yml
fi

echo "Running GeoScenario server within miniforge gss environement"

cd $REPO_DIR
(set -x;
    $MAMBA_EXE -n gss run python3 GSServer.py -s scenarios/coretest_scenarios/straightdrive.gs.osm
)

echo "Execute:"
echo "micromamba/mamba -n gss run python3 geoscenarioserver/GSServer.py -s <scenario path>"
echo ""
