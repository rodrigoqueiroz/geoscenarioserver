#!/bin/bash
set -e

install_python_dependencies()
{
    echo ""
    echo "Installing Python3.8 and other packages"
    echo ""
    # Ensure we have Python3.8 and pip3
    sudo apt-get install -qq python3.8 python3.8-dev python3-tk python3-pip

    echo ""
    echo "Installing the required dependencies for GeoScenario Server for Python3.8"
    echo ""
    python3.8 -m pip -q install numpy scipy glog matplotlib py_trees antlr4-python3-runtime tk sysv-ipc antlr-denter
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
    sudo apt-get -qq install libpugixml-dev "ros-${ROS_VERSION}-mrt-cmake-modules" "ros-${ROS_VERSION}-lanelet2"
}

install_python_dependencies
install_lanelet2_binary

echo ""
echo "Successfully installed GeoScenario Server dependencies."
echo ""
