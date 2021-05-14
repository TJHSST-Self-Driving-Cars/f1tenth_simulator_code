#!/bin/bash

source /catkin_ws/devel/setup.bash
source ${RIDERS_PATH}/setup.sh

# start display functionality at :0
export DISPLAY=:0
ride_display_start "${DISPLAY}"

#

# start roscore & rviz
nohup rviz -d /f1tenth.rviz > /tmp/rviz.log &

# TODO: export Display here, so instead of RVIZ we see the actual GYM screen output
roslaunch --wait f1tenth_gym_ros gym_bridge.launch