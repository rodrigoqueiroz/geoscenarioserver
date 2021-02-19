#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

install_python_dependencies()
{
    echo ""
    echo "Installing Python3.8 and other packages"
    echo ""
    # Ensure we have Python3.8 and pip3
    sudo apt-get install -qq python3.8 python3.8-dev python3-tk python3-pip

    echo ""
    echo "Installing the required dependencies for catkin for Python3.8"
    echo ""
    python3.8 -m pip -q install catkin_pkg rospkg

    echo ""
    echo "Installing the required dependencies for GeoScenario Server for Python3.8"
    echo ""
    python3.8 -m pip -q install numpy scipy glog matplotlib py_trees antlr4-python3-runtime tk sysv-ipc antlr-denter
}

install_lanelet2_python38()
{
    echo ""
    echo "Installing Lanelet2 library from source for Python3.8..."
    echo ""
    cd $REPO_DIR
    # In the case the repository was not cloned recursively
    git submodule update --init

    # temporarily switch to Python3.8 for the catkin build
    alias python=/usr/bin/python3.8
    alias python3=/usr/bin/python3.8

    mkdir -p $REPO_DIR/catkin_ws/src
    ln -sfn $REPO_DIR/Lanelet2 $REPO_DIR/catkin_ws/src/Lanelet2

    source "$SCRIPT_DIR/catkin_utils.bash"

    local CATKIN_DIR="$REPO_DIR/catkin_ws"

    create_catkin_workspace $CATKIN_DIR

    build_catkin_workspace $CATKIN_DIR --clean

    # Allow for errors in order not to exit this script
    set +e
    set +o pipefail
    fix_paths_in_install_space $CATKIN_DIR
    set -e
    set -o pipefail
}

install_lanelet2_binary()
{
    echo ""
    echo "Installing Lanelet2 library from ROS binaries..."
    echo ""
    if which rosversion > /dev/null && [[ $(rosversion -d) == "<unknown>" ]]; then
        echo "No ROS distribution was found. Please ensure that setup.bash is sourced."
        exit 1
    fi
    ROS_VERSION=$(rosversion -ds)
    sudo apt-get -qq install libpugixml-dev "ros-${ROS_VERSION}-mrt-cmake-modules" "ros-${ROS_VERSION}-lanelet2"
}

install_python_dependencies
install_lanelet2_binary
install_lanelet2_python38

echo ""
echo "Successfully installed GeoScenario Server dependencies."
echo ""
