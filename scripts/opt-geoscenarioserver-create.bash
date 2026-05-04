#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname ${SCRIPT_DIR})
CHANNEL_TEST_DIR=${REPO_DIR}/test/conda-channel-test
VERSION=${1:-"0.1.3"}

# ensure pixi pack and unpack are available
pixi global install pixi-pack pixi-unpack

cd ${CHANNEL_TEST_DIR}
set -x
git clean -fdx
pixi install
pixi pack -p linux-64
pixi unpack ${CHANNEL_TEST_DIR}/environment.tar -o /opt/geoscenarioserver/
rm ${CHANNEL_TEST_DIR}/environment.tar
tar -caf opt-geoscenarioserver-${VERSION}.tar.zst /opt/geoscenarioserver/

echo "${CHANNEL_TEST_DIR}/opt-geoscenarioserver-${VERSION}.tar.zst is ready for upload"
