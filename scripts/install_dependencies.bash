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
    sudo apt-get install -qq python3.8 python3.8-dev python3-tk python3-pip python3-pil python3-pil.imagetk

    echo ""
    echo "Installing the required dependencies for catkin for Python3.8"
    echo ""
    python3.8 -m pip -q install catkin_pkg rospkg

    echo ""
    echo "Installing the required dependencies for GeoScenario Server for Python3.8"
    echo ""
    python3.8 -m pip -q install numpy scipy glog matplotlib py_trees antlr4-python3-runtime==4.7.2 tk sysv-ipc antlr-denter
}

install_lanelet2_python38()
{
    echo ""
    echo "Installing Lanelet2 library from source for Python3.8..."
    echo ""
    # Ensure catkin is installed.
    ROS_VERSION=$(rosversion -ds)
    if [[ "$ROS_VERSION" == "noetic" ]]; then
    	sudo apt-get -qq install python3-catkin-tools
    elif [[ "$ROS_VERSION" == "melodic" ]]; then
    	sudo apt-get -qq install python-catkin-tools
    else
    	echo "The ROS distribution $ROS_VERSION is currently unsupported!"
    fi
    
    cd $REPO_DIR
    # In the case the repository was not cloned recursively
    git submodule update --init

    # temporarily switch to Python3.8 for the catkin build
    alias python=/usr/bin/python3.8
    alias python3=/usr/bin/python3.8

    mkdir -p $REPO_DIR/catkin_ws/src
    ln -sfn $REPO_DIR/Lanelet2 $REPO_DIR/catkin_ws/src/Lanelet2
    # for noetic: package mrt_cmake_modules has to be source build.
    if [[ "$ROS_VERSION" == "noetic" ]]; then
    	git clone https://github.com/KIT-MRT/mrt_cmake_modules.git mrt_cmake_modules
    	ln -sfn $REPO_DIR/mrt_cmake_modules $REPO_DIR/catkin_ws/src/mrt_cmake_modules
    fi

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
    if ! which rosversion > /dev/null || [[ $(rosversion -sd) == "<unknown>" ]]; then
        echo "No ROS distribution was found. Please ensure that setup.bash is sourced."
        exit 1
    fi
    ROS_VERSION=$(rosversion -ds)
    sudo apt-get -qq install libpugixml-dev
   
# Under Noetic distribution, mrt-cmake-modules does not include the newest version of the findboostpython script, thus reporting error when source build lanelet2-python. Should be source built. (Ref:https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/221)
if [[ "$ROS_VERSION" == "melodic" ]]; then
     sudo apt-get -qq install "ros-${ROS_VERSION}-mrt-cmake-modules" "ros-${ROS_VERSION}-lanelet2"
     fi
}

install_python_dependencies
install_lanelet2_binary
install_lanelet2_python38
echo ""
echo "Successfully installed GeoScenario Server dependencies."
echo ""
