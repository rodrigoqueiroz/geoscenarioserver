#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

activate_venv()
{
    source $REPO_DIR/.venv/bin/activate
}

install_python_dependencies()
{
    # Ensure we have Python3.8 and pip3
    sudo apt-get install -qq python3.8 python3.8-dev python3.8-venv python3-tk python3-pip
    # Create a virtual environment
    cd $REPO_DIR
    python3.8 -m venv .venv
    # Install the required dependencies for GeoScenario Server
    python3.8 -m pip -q install numpy scipy glog matplotlib py_trees antlr4-python3-runtime tk sysv-ipc antlr-denter
}

install_lanelet2_python38()
{
    cd $REPO_DIR
    # In the case the repository was not cloned recursively
    git submodule update --init
    activate_venv
    mkdir -p $REPO_DIR/catkin_ws/src
    ln -sfn $REPO_DIR/Lanelet2 $REPO_DIR/catkin_ws/src/Lanelet2
    cd $REPO_DIR/catkin_ws
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_VERSION=3.8
    pip3 -q install catkin_pkg rospkg
    catkin build --workspace $REPO_DIR/catkin_ws
}

install_lanelet2_binary()
{
    if which rosversion > /dev/null && [[ $(rosversion -d) == "<unknown>" ]]; then
        echo "No ROS distribution was found. Please be sure that setup.bash is sourced."
        exit 1
    fi

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
        exit 1
    fi
}

install_python_dependencies
install_lanelet2_binary
install_lanelet2_python38
