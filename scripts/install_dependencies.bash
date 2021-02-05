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
    # ensure we have pip3
    sudo apt install python3-pip python3-tk python3-sysv-ipc
    sudo pip3 install numpy scipy glog matplotlib
}

install_lanelet2()
{
    sudo apt-get -qq install libpugixml-dev
    UBUNTU_DISTRO=$(lsb_release -s -c)
    if [[ "$UBUNTU_DISTRO" == "xenial" ]]; then
        # For Xenial, get the submodules and will build
        git submodule update --init --recursive
    elif [[ "$UBUNTU_DISTRO" == "bionic" ]]; then
        sudo apt-get -qq install "ros-melodic-mrt-cmake-modules" "ros-melodic-lanelet2"
    elif [[ "$UBUNTU_DISTRO" == "focal" ]]; then
        sudo apt-get -qq install "ros-noetic-mrt-cmake-modules" "ros-noetic-lanelet2"
    else
        echo "Unsupported ROS "
    fi
}


main
