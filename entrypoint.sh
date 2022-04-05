#!/bin/bash -e

ROS_FLAGS=""
if [[ ${SIMULATION+x} != "" ]]; then
    ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

exec ros-with-env ros2 launch octomap_server2 octomap_server.py ${ROS_FLAGS}
