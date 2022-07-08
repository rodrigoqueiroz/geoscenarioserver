#!/usr/bin/env bash

# Check if sourced or executed
if [[ ${BASH_SOURCE[0]} == "${0}" ]]; then
    echo "ERROR: This script should be sourced not executed."
    exit
fi

create_catkin_workspace()
{
    local CATKIN_DIR=$1
    cd $CATKIN_DIR

    # Set DESTDIR to absolute path of install space
    export DESTDIR="$CATKIN_DIR/install"

    echo ""
    echo "Configuring Catkin workspace..."
    echo ""

    # Suppress output from config, since it is shown again on build
    catkin init --workspace .
    catkin config --extend /opt/ros/$ROS_DISTRO \
        --install-space /opt/ros/lanelet2 \
        --install \
        --mkdirs  \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DPYTHON_VERSION=3.8 \
                     -DTARGET_INSTALL_DIR="/opt/ros/lanelet2"

    echo ""
    echo "Successfully created Catkin workspace"
    echo ""
}

build_catkin_workspace()
{
    local CATKIN_DIR=$1
    local CLEAN_ARG=$2

    echo ""
    echo "Building catkin workspace $CATKIN_DIR..."
    echo ""

    # Set DESTDIR to absolute path of install space
    export DESTDIR="$CATKIN_DIR/install"

    cd $CATKIN_DIR

    if [[ "$CLEAN_ARG" == "--clean" ]]; then
      catkin clean -y
    fi

    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        catkin build
    else
        catkin build --no-status --continue-on-failure
    fi

    echo ""
    echo "Successfully built $CATKIN_DIR"
    echo ""
}

fix_paths_in_install_space()
{
    local CATKIN_DIR=$1

    INSTALL_PATH="$CATKIN_DIR/install/"
    echo ""
    echo "Fixing paths in the install space ${INSTALL_PATH}..."
    echo ""

    echo "1. Replacing $INSTALL_PATH with /..."
    NEW_PATH="/"
    find "$INSTALL_PATH" -type f \
           \( -name "*.cmake" -o -name "*.pc" -o -name "*.sh" -o -name _setup_util.py \) \
           -print0 | xargs -0 sed --in-place "s|$INSTALL_PATH|$NEW_PATH|g"

    DEVEL_PATH="$CATKIN_DIR/devel/"
    NEW_PATH="/"
    echo "2. Replacing $DEVEL_PATH in SOURCES.txt with /..."
    find "$INSTALL_PATH" -type f \
           -name SOURCES.txt \
           -print0 | xargs -0 sed --in-place "s|$DEVEL_PATH|$NEW_PATH|g"

     echo ""
     echo "Successfully fixed paths in the install space ${INSTALL_PATH}"
     echo ""
}

remove_catkin_workspace()
{
    local CATKIN_DIR=$1
    rm -rf "$CATKIN_DIR/.catkin_tools/" \
           "$CATKIN_DIR/build/" \
           "$CATKIN_DIR/devel/" \
           "$CATKIN_DIR/install/" \
           "$CATKIN_DIR/logs/"
}
