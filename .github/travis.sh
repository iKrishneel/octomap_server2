#!/usr/bin/env bash

echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

apt-get update && apt-get install -y \
 			  libflann-dev \
			  liboctomap-dev \
 			  libpcl-dev

if [ -z ${CI_SOURCE_DIR} ];
then
    (cd ${CI_SOURCE_DIR}; git log --oneline | head -10)
fi

if [ -z ${REPO_NAME} ];
then
    export REPO_NAME=octomap_server2
fi

if [ -z ${CI_SOURCE_PATH} ];
then
    export CI_SOURCE_PATH=/home/src/$REPO_NAME
fi

export WORKDIR=${HOME}/ros2/${ROS_DISTRO}/src

if [ ! -d $WORKDIR ];
then
    mkdir -p $WORKDIR 
fi

cd $WORKDIR/..
mkdir -p src/$REPO_NAME
cp -r ${CI_SOURCE_PATH} src/

echo `pwd`
ls
echo $CI_SOURCE_PATH

vcs import ./src/$REPO_NAME < src/${REPO_NAME}/deps.repos
colcon build --symlink-install
