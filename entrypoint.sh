#!/bin/bash -e

source /opt/ros/galactic/setup.bash

ROS_FLAGS=""
if [[ ${SIMULATION+x} != "" ]]; then
    ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

ros2 launch octomap_server2 octomap_server.py ${ROS_FLAGS}
