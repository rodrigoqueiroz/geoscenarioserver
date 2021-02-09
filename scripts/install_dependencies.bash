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
    export DESTDIR="$REPO_DIR/catkin_ws/install"
    catkin init --workspace $REPO_DIR/catkin_ws
    catkin config --workspace $REPO_DIR/catkin_ws \
                  --install \
                  -i /opt/ros/lanelet2 \
                  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.8 \
                  -DTARGET_INSTALL_DIR="/opt/ros/lanelet2" > /dev/null
    pip3 -q install catkin_pkg rospkg
    catkin build --workspace $REPO_DIR/catkin_ws
}

install_lanelet2_binary()
{
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
