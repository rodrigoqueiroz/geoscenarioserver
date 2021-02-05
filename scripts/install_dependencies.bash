#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

main()
{
    install_python_dependencies
    install_lanelet2
}

install_python_dependencies()
{
    # Ensure we have Python3.8 and pip3
    sudo apt-get install -qq python3.8 python3.8-dev python3.8-venv python3-pip
    # Install the required dependencies for GeoScenario Server
    pip3 -q install numpy scipy glog matplotlib py_trees antlr4-python3-runtime tk sysv-ipc
}

install_lanelet2()
{
    sudo apt-get -qq install libpugixml-dev
    UBUNTU_DISTRO=$(lsb_release -s -c)
    if [[ "${UBUNTU_DISTRO}" == "xenial" ]]; then
        # For Xenial, get the submodules and will build
        git submodule update --init --recursive
    elif [[ "${UBUNTU_DISTRO}" == "bionic" ]]; then
        sudo apt-get -qq install "ros-melodic-mrt-cmake-modules" "ros-melodic-lanelet2"
    elif [[ "${UBUNTU_DISTRO}" == "focal" ]]; then
        sudo apt-get -qq install "ros-noetic-mrt-cmake-modules" "ros-noetic-lanelet2"
    else
        echo "Unsupported Ubuntu version ${UBUNTU_DISTRO}."
    fi
}

main
