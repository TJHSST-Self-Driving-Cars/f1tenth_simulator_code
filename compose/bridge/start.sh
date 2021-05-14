#!/bin/bash

source /catkin_ws/devel/setup.bash
source ${RIDERS_PATH}/setup.sh

# start display functionality at :0
export DISPLAY=:1
ride_display_start "${DISPLAY}"
# TODO: export Display here, so instead of RVIZ we see the actual GYM screen output

roslaunch --wait f1tenth_gym_ros gym_bridge.launch