#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /root/tb3_stage_explore_ws/install/setup.bash

python3 /root/tb3_stage_explore_ws/src/tb3_stage_explore/python/singlerun.py "$1"
