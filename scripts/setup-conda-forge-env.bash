#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

print_usage() {
    echo "Usage:"
    echo "  $ bash setup_conda-forge_env.bash [-r|--ros2] [-t|--test-run] [-h|--help]"
    echo "    -r|--ros2       install ROS2 humble and build tools into the environment 'gss'; build the ROS2 client"
    echo "    -t|--test-run   start GeoScenarioServer within the environment 'gss'"
    echo "    -h|--help       display usage instructions and exit"
    echo ""
}

ARG_ROS2=
ARG_TEST_RUN=
for arg in "$@"; do
    case $arg in
        -r|--ros2)
            ARG_ROS2="true"
            ;;
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
    echo "Creating conda environment 'gss'..."
    $MAMBA_EXE env create --yes --quiet --file ${SCRIPT_DIR}/conda-environment.yml
    if [[ $? == 0 ]]; then
        # conda-forge tk does not support TrueType fonts
        # remove it so that we can use the one from pip
        $MAMBA_EXE -n gss remove --force --yes --quiet tk
        echo "The environment created successfully."
    else 
        echo "Environment not created. Exiting..."
        exit 1
    fi
else
    echo "The environment 'gss' already exists; remove it first to recreate." 
fi

if [[ ${ARG_ROS2} == "true" ]]; then
    echo "Installing ROS2 into the environment 'gss'..."
    $MAMBA_EXE -n gss install --yes --quiet -c conda-forge -c robostack-staging ros-humble-desktop ros-humble-geographic-msgs compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
    # conda-forge tk gets reinstalled
    # remove it again
    $MAMBA_EXE -n gss remove --force --yes --quiet tk
    if [[ $? == 0 ]]; then
        echo "ROS2 installed successfully. Building the GSS ROS2 client..."
        echo ""
        mkdir -p ${REPO_DIR}/colcon_ws/src
        ln -sfn ${REPO_DIR}/clients/ros2_client ${REPO_DIR}/colcon_ws/src/ros2_client
        cd ${REPO_DIR}/colcon_ws
        $MAMBA_EXE -n gss run colcon build
        echo ""
        echo "-------------------------------"
        echo "To run the ROS2 client, execute"
        echo "  $ $(basename $MAMBA_EXE) -n gss run bash -c 'source ${REPO_DIR}/colcon_ws/install/setup.bash && ros2 run geoscenario_client geoscenario_client'"
    else
        echo "Failed to install ROS2 and build the GSS ROS2 client."
    fi
fi

if [[ ${ARG_TEST_RUN} == "true" ]]; then
    echo "Running GeoScenario server within conda forge environement..."

    cd $REPO_DIR
    (set -x;
        $MAMBA_EXE -n gss run python3 GSServer.py -s scenarios/coretest_scenarios/straightdrive.gs.osm
    )
fi

echo ""
echo "---------------------------------"
echo "To run GeoScenarioServer, execute"
echo "  $ $(basename $MAMBA_EXE) -n gss run python3 GSServer.py -s <scenario_file_path>.osm"
echo ""
