#!/bin/bash -e

_term() {
	# FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
	pattern="component_container_mt"

	pid_value="$(ps -ax | grep $pattern | grep -v grep | awk '{ print $1 }')"
	if [ "$pid_value" != "" ]; then
		pid=$pid_value
		echo "Send SIGINT to pid $pid"
	else
		pid=1
		echo "Pattern not found, send SIGINT to pid $pid"
	fi
	kill -s SIGINT $pid
}
trap _term SIGTERM

ROS_FLAGS=""
if [[ ${SIMULATION+x} != "" ]]; then
	ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

ros-with-env ros2 launch octomap_server2 octomap_server.py ${ROS_FLAGS} &
child=$!

echo "Waiting for pid $child"
wait $child
RESULT=$?

if [ $RESULT -ne 0 ]; then
		echo "ERROR: Octomap server failed with code $RESULT" >&2
		exit $RESULT
else
		echo "INFO: Octomap server finished successfully, but returning 125 code for docker to restart properly." >&2
		exit 125
fi
