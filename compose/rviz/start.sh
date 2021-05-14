#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${RIDERS_PATH}/setup.sh

# start display functionality at :0
export DISPLAY=:0
ride_display_start "${DISPLAY}"

# start roscore & rviz
nohup rviz -d /f1tenth.rviz > /tmp/rviz.log &

tail -f /dev/null;