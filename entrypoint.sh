#!/bin/bash -e

ROS_FLAGS=""
if [[ ${SIMULATION+x} != "" ]]; then
    ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

exec ros-with-env ros2 launch octomap_server2 octomap_server.py ${ROS_FLAGS}

RESULT=$?
if [ $RESULT -ne 0 ]; then
		echo "ERROR: Octomap server failed with code $RESULT" >&2
		exit $RESULT
else
		echo "INFO: Octomap server finished successfully, but returning 125 code for docker to restart properly." >&2
		exit 125
fi
