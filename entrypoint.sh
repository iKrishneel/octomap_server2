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
# Use SIGTERM or TERM, does not seem to make any difference.
trap _term TERM

ROS_FLAGS=""
if [[ ${SIMULATION+x} != "" ]]; then
	ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

ros-with-env ros2 launch octomap_server2 octomap_server.py ${ROS_FLAGS} &
child=$!

echo "Waiting for pid $child"
# * Calling "wait" will then wait for the job with the specified by $child to finish, or for any signals to be fired.
#   Due to "or for any signals to be fired", "wait" will also handle SIGTERM and it will shutdown before
#   the node ends gracefully.
#   The solution is to add a second "wait" call and remove the trap between the two calls.
# * Do not use -e flag in the first wait call because wait will exit with error after catching SIGTERM.
set +e
wait $child
set -e
trap - TERM
wait $child
RESULT=$?

if [ $RESULT -ne 0 ]; then
		echo "ERROR: Octomap server failed with code $RESULT" >&2
		exit $RESULT
else
		echo "INFO: Octomap server finished successfully, but returning 125 code for docker to restart properly." >&2
		exit 125
fi
