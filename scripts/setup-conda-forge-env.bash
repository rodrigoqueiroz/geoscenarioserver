#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

print_usage() {
    echo "Usage:"
    echo "  $ bash setup_conda-forge_env.bash [-t|--test-run] [-h|--help]"
    echo "    -t|--test-run   start GeoScenarioServer within the environment 'gss'"
    echo "    -h|--help       show these instructions"
    echo ""
}
ARG_TEST_RUN=

for arg in "$@"; do
    case $arg in
        -t|--test-run)
            ARG_TEST_RUN="true"
            ;;
        -h|--help)
            echo ""
            echo "Create a conda-forge environment called gss for running GeoScenarioServer"
            echo ""
            print_usage
            exit 0
            ;;
        *)
            echo "Invalid argument '$arg'"
            echo ""
            echo "Usage: "
            print_usage
            exit 1
    esac
done

# Use MAMBA_EXE variable so that it works with either micromamba or mamba
if [[ -z $MAMBA_EXE ]]; then
    echo "Mamba or micromamba not installed or not activated."
    echo "Follow the instructions for installing either"
    echo "   micromamba (recommended): https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html"
    echo "   or mamba:                 https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html"
    exit 1
fi

if ! $MAMBA_EXE env list | grep -q gss; then
    echo "Creating conda forge 'gss' environment..."
    $MAMBA_EXE env create --yes --quiet --file ${SCRIPT_DIR}/conda-environment.yml
    if [[ $? == 0 ]]; then 
        echo "The environment created successfully."
    else 
        echo "Environment not created. Exiting..."
        exit 1
    fi
else
    echo "The environment 'gss' already exists. Remove first to recreate." 
fi

if [[ ${ARG_TEST_RUN} == "true" ]]; then
    echo "Running GeoScenario server within conda forge environement..."

    cd $REPO_DIR
    (set -x;
        $MAMBA_EXE -n gss run python3 GSServer.py -s scenarios/coretest_scenarios/straightdrive.gs.osm
    )
fi

echo ""
echo "To run GeoScenarioServer, execute"
echo "  $ $(basename $MAMBA_EXE) -n gss run python3 GSServer.py -s <scenario_file_path>.osm"
echo ""
