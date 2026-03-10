#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname ${SCRIPT_DIR})
CHANNEL_TEST_DIR=${REPO_DIR}/test/conda-channel-test

cd ${CHANNEL_TEST_DIR}
set -x
git clean -fdx
pixi install
pixi pack -p linux-64
pixi unpack ${CHANNEL_TEST_DIR}/environment.tar -o /opt/geoscenarioserver/
rm ${CHANNEL_TEST_DIR}/environment.tar
tar -I 'zstd -T$(nproc) -19' -cpf geoscenarioserver.tar.zstd /opt/geoscenarioserver/

echo "geoscenarioserver.tar.zstd is ready for upload"