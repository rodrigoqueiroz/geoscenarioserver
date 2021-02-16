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
    cd $REPO_DIR/catkin_ws
    catkin init --workspace .
    export DESTDIR="$REPO_DIR/catkin_ws/install"
    catkin config --extend /opt/ros/melodic \
                  --merge-devel \
                  --install-space /opt/ros/lanelet2 \
                  --install \
                  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.8 \
                  -DTARGET_INSTALL_DIR="/opt/ros/lanelet2" > /dev/null
    catkin build
    echo ""
    echo "The final catkin config:"
    echo ""
    catkin config
    echo ""
    echo "Fixing paths in the binaries:"
    echo ""
    ORIGINAL_PATH="$REPO_DIR/catkin_ws/install/"
    NEW_PATH="/"
    find "$ORIGINAL_PATH" -type f \
           \( -name "*.cmake" -o -name "*.pc" -o -name "*.sh" -o -name _setup_util.py \) \
           -print0 | xargs -0 sed -i "s|$ORIGINAL_PATH|$NEW_PATH|g"
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
