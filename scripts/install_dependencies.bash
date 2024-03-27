#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

echo ""
echo "Upgrading packages"
echo ""
sudo apt-get update && sudo apt-get upgrade -y

echo ""
echo "Installing Python3 packages"
echo ""
# Ensure we have Python3 and pip3
sudo apt-get install -qq python3 python3-dev python3-tk python3-pip python3-pil python3-pil.imagetk

echo ""
echo "Installing GraphViz"
echo ""
# dot is required
sudo apt-get install -qq graphviz

echo ""
echo "Installing the required dependencies for GeoScenario Server for Python3"
echo ""
python3 -m pip install -r ${REPO_DIR}/requirements.txt

echo ""
echo "Successfully installed GeoScenario Server dependencies."
echo ""
