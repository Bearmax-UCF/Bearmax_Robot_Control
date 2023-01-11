#!/bin/bash
source /opt/ros/humble/setup.bash
if [ -d install ]; then
    source install/local_setup.bash
    source install/setup.bash
fi
clear
