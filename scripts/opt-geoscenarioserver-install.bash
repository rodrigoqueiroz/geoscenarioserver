#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -f /opt/geoscenarioserver/activate.sh ]; then
    echo "GeoScenarioServer found in /opt/geoscenarioserver"
    echo "Remove and re-run the script to download and install the latest version"
else
    echo "Downloading and installing GeoScenarioServer to /opt/geoscenarioserver"
    NAME=opt-geoscenarioserver-latest.tar.zstd
    curl -L https://wiselab.uwaterloo.ca/wise-sim/${NAME} -o ${SCRIPT_DIR}/${NAME}
    tar -I zstdmt -xf ${SCRIPT_DIR}/${NAME} -C /
    rm ${SCRIPT_DIR}/${NAME}
fi
