#!/bin/sh
set -e

# Create config directory and copy config file
mkdir -p ~/.grsim
cp /tmp/grsim_config.xml ~/.grsim/config.xml

# Start Xvfb in background
Xvfb :99 -screen 0 1024x768x24 &
sleep 2

# Start grSim
exec grSim -H
