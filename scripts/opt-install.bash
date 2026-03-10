#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SUITE_DIR=$(dirname "$SCRIPT_DIR")

if [ -f /opt/geoscenarioserver/activate.sh ]; then
    echo "GeoScenarioServer found in /opt/geoscenarioserver"
    echo "Remove and re-run the script to download and install the latest version"
else
    echo "Downloading and installing GeoScenarioServer to /opt/geoscenarioserver"
    curl -L https://wiselab.uwaterloo.ca/wise-sim/opt-geoscenarioserver-latest.tar.zstd  -o ${SCRIPT_DIR}/opt-geoscenarioserver.tar.zstd
    tar -I zstdmt -xf ${SCRIPT_DIR}/opt-geoscenarioserver.tar.zstd -C /opt/
    rm ${SCRIPT_DIR}/opt-geoscenarioserver.tar.zstd
fi
